/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package frc.robot.subsystems;

import java.util.function.Supplier;

import monologue.Logged;
import monologue.Annotations.Log;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

import frc.lib.subsystem.Subsystem;
import frc.lib.swerve.*;
import frc.lib.swerve.SwerveRequest.SwerveControlRequestParameters;
import frc.lib.swerve.generated.TunerConstants;
import frc.robot.Constants;
import frc.robot.OI;

/**
 * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API.
 * <p>
 * This class handles the kinematics, configuration, and odometry of a
 * swerve drive utilizing CTR Electronics devices. We recommend
 * that users use the Swerve Mechanism Generator in Tuner X to create
 * a template project that demonstrates how to use this class.
 * <p>
 * This class will construct the hardware devices internally, so the user
 * only specifies the constants (IDs, PID gains, gear ratios, etc).
 * Getters for these hardware devices are available.
 * <p>
 * If using the generator, the order in which modules are constructed is
 * Front Left, Front Right, Back Left, Back Right. This means if you need
 * the Back Left module, call {@code getModule(2);} to get the 3rd index
 * (0-indexed) module, corresponding to the Back Left module.
 */
public class SwerveDrivetrain extends Subsystem {
    private static SwerveDrivetrain instance;

    public static SwerveDrivetrain getInstance() {
        if (instance == null) {
            instance = new SwerveDrivetrain(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft,
                    TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        }
        return instance;
    }

    // Drive Mode Selections
    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        TARGET,
        AUTONOMOUS
    }

    private SwerveRequest.FieldCentric field_centric;
    private SwerveRequest.RobotCentric robot_centric;
    private SwerveRequest.FieldCentricFacingAngle target_facing;

    // Robot Hardware
    // private final Pigeon2 pigeon_imu;
    private final SwerveModule[] swerve_modules;
    private final AHRS ahrs;

    // Subsystem data class
    private SwerveDriverainPeriodicIo io_;

    // Drivetrain config
    final SwerveDriveKinematics kinematics;
    private final Translation2d[] module_locations;

    // Drive requests
    private SwerveRequest.ApplyChassisSpeeds auto_request;
    private SwerveRequest request_to_apply;
    private SwerveControlRequestParameters request_parameters;

    // NT publishers
    private StructArrayPublisher<SwerveModuleState> current_state_pub, requested_state_pub;

    /**
     * Constructs a SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * @param driveTrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public SwerveDrivetrain(
            SwerveDrivetrainConstants driveTrainConstants,
            SwerveModuleConstants... modules) {

        // make new io instance
        io_ = new SwerveDriverainPeriodicIo();

        // // Setup the Pigeon IMU
        // pigeon_imu = new Pigeon2(driveTrainConstants.Pigeon2Id,
        // driveTrainConstants.CANbusName[0]);
        // pigeon_imu.optimizeBusUtilization();
        ahrs = new AHRS(SerialPort.Port.kUSB);

        // Begin configuring swerve modules
        module_locations = new Translation2d[modules.length];
        swerve_modules = new SwerveModule[modules.length];
        io_.module_positions = new SwerveModulePosition[modules.length];
        io_.current_module_states_ = new SwerveModuleState[modules.length];
        io_.requested_module_states_ = new SwerveModuleState[modules.length];

        // Construct the swerve modules
        for (int i = 0; i < modules.length; i++) {
            SwerveModuleConstants module = modules[i];
            swerve_modules[i] = new SwerveModule(module, driveTrainConstants.CANbusName[i],
                    driveTrainConstants.SupportsPro);
            module_locations[i] = new Translation2d(module.LocationX, module.LocationY);
            io_.module_positions[i] = swerve_modules[i].getPosition(true);
            io_.current_module_states_[i] = swerve_modules[i].getCurrentState();
            io_.requested_module_states_[i] = swerve_modules[i].getRequestedState();

        }
        kinematics = new SwerveDriveKinematics(module_locations);

        // Drive mode requests
        field_centric = new SwerveRequest.FieldCentric().withIsOpenLoop(true)
                .withDeadband(Constants.DrivetrainConstants.MaxSpeed * 0.1)
                .withRotationalDeadband(Constants.DrivetrainConstants.MaxAngularRate * 0.01);
        robot_centric = new SwerveRequest.RobotCentric().withIsOpenLoop(true)
                .withDeadband(Constants.DrivetrainConstants.MaxSpeed * 0.1)
                .withRotationalDeadband(Constants.DrivetrainConstants.MaxAngularRate * 0.01);
        target_facing = new SwerveRequest.FieldCentricFacingAngle().withIsOpenLoop(true)
                .withDeadband(Constants.DrivetrainConstants.MaxSpeed * 0.1)
                .withRotationalDeadband(Constants.DrivetrainConstants.MaxAngularRate * 0.01);
        auto_request = new SwerveRequest.ApplyChassisSpeeds();
        request_parameters = new SwerveControlRequestParameters();
        request_to_apply = new SwerveRequest.Idle();

        // NT Publishers
        requested_state_pub = NetworkTableInstance.getDefault()
                .getStructArrayTopic("module_states/requested", SwerveModuleState.struct).publish();
        current_state_pub = NetworkTableInstance.getDefault()
                .getStructArrayTopic("module_states/current", SwerveModuleState.struct).publish();

        SmartDashboard.putData("Set Wheel Offsets", Commands.runOnce(() -> tareEverything(), this));
    }

    @Override
    public void reset() {
        // 4 signals for each module + 2 for Pigeon2
        for (int i = 0; i < swerve_modules.length; ++i) {
            BaseStatusSignal.setUpdateFrequencyForAll(100, swerve_modules[i].getSignals());
            swerve_modules[i].optimizeCan();
        }
        // BaseStatusSignal[] imuSignals = { pigeon_imu.getYaw() };
        // BaseStatusSignal.setUpdateFrequencyForAll(100, imuSignals);
        // pigeon_imu.optimizeBusUtilization();

        configurePathPlanner();
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        for (int i = 0; i < swerve_modules.length; ++i) {
            io_.module_positions[i] = swerve_modules[i].getPosition(true);
            io_.current_module_states_[i] = swerve_modules[i].getCurrentState();
            io_.requested_module_states_[i] = swerve_modules[i].getRequestedState();
        }
        io_.driver_joystick_leftX_ = OI.getDriverJoystickLeftX();
        io_.driver_joystick_leftY_ = OI.getDriverJoystickLeftY();
        io_.driver_joystick_rightX_ = OI.getDriverJoystickRightX();

        // io_.robot_yaw_ =
        // Rotation2d.fromRadians(MathUtil.angleModulus(-pigeon_imu.getAngle() * Math.PI
        // / 180));
        // io_.robot_yaw_ = Rotation2d.fromRadians(MathUtil.angleModulus(0 * Math.PI /
        // 180));

        io_.robot_yaw_ = Rotation2d.fromRadians(MathUtil.angleModulus(-ahrs.getAngle() * Math.PI / 180));

        io_.chassis_speeds_ = kinematics.toChassisSpeeds(io_.current_module_states_);

    }

    @Override
    public void updateLogic(double timestamp) {
        switch (io_.drive_mode_) {
            case ROBOT_CENTRIC:
                setControl(robot_centric
                        // Drive forward with negative Y (forward)
                        .withVelocityX(-io_.driver_joystick_leftY_ * Constants.DrivetrainConstants.MaxSpeed)
                        // Drive left with negative X (left)
                        .withVelocityY(-io_.driver_joystick_leftX_ * Constants.DrivetrainConstants.MaxSpeed)
                        // Drive counterclockwise with negative X (left)
                        .withRotationalRate(
                                io_.driver_joystick_rightX_ * Constants.DrivetrainConstants.MaxAngularRate));
                break;
            case FIELD_CENTRIC:
                setControl(field_centric
                        // Drive forward with negative Y (forward)
                        .withVelocityX(-io_.driver_joystick_leftY_ * Constants.DrivetrainConstants.MaxSpeed)
                        // Drive left with negative X (left)
                        .withVelocityY(-io_.driver_joystick_leftX_ * Constants.DrivetrainConstants.MaxSpeed)
                        // Drive counterclockwise with negative X (left)
                        .withRotationalRate(
                                io_.driver_joystick_rightX_ * Constants.DrivetrainConstants.MaxAngularRate));
                break;
            case TARGET:
                setControl(target_facing
                        // Drive forward with negative Y (forward)
                        .withVelocityX(-io_.driver_joystick_leftY_ * Constants.DrivetrainConstants.MaxSpeed)
                        // Drive left with negative X (left)
                        .withVelocityY(-io_.driver_joystick_leftX_ * Constants.DrivetrainConstants.MaxSpeed)
                        //
                        .withTargetDirection(io_.target_rotation_));
                break;
            case AUTONOMOUS:
                setControl(robot_centric
                        // Drive forward with negative Y (forward)
                        .withVelocityX(-0.5)
                        // Drive left with negative X (left)
                        .withVelocityY(0)
                        //
                        .withRotationalRate(0));
                break;
            default:
                // yes these dont do anything for auto...
                break;
        }

        /* And now that we've got the new odometry, update the controls */
        request_parameters.currentPose = new Pose2d(0, 0, io_.robot_yaw_)
                .relativeTo(new Pose2d(0, 0, io_.field_relative_offset_));
        request_parameters.kinematics = kinematics;
        request_parameters.swervePositions = module_locations;
        request_parameters.updatePeriod = timestamp - request_parameters.timestamp;
        request_parameters.timestamp = timestamp;
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        request_to_apply.apply(request_parameters, swerve_modules);
    }

    @Override
    public void outputTelemetry(double timestamp) {
        current_state_pub.set(io_.current_module_states_);
        requested_state_pub.set(io_.requested_module_states_);

        SmartDashboard.putNumber("Yaw", io_.robot_yaw_.getDegrees());
        SmartDashboard.putNumber("Field Centric Offset", io_.field_relative_offset_.getDegrees());
        SmartDashboard.putBoolean("Is Field Centric", (io_.drive_mode_ == DriveMode.FIELD_CENTRIC));
    }

    /**
     * Configures the PathPlanner AutoBuilder
     */
    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : module_locations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                PoseEstimator.getInstance()::getRobotPose, // Supplier of current robot pose
                PoseEstimator.getInstance()::setRobotOdometry, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(auto_request.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                              // robot
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        5,
                        driveBaseRadius,
                        new ReplanningConfig(false, false),
                        0.008), // faster period than default

                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return Commands.run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return io_.chassis_speeds_;
    }

    /**
     * Takes the current orientation of the robot and makes it X forward for
     * field-relative maneuvers.
     */
    public void seedFieldRelative() {
        io_.field_relative_offset_ = io_.robot_yaw_;
    }

    /**
     * Applies the specified control request to this swerve drivetrain.
     *
     * @param request Request to apply
     */
    public void setControl(SwerveRequest request) {
        request_to_apply = request;
    }

    /**
     * Zero's this swerve drive's odometry entirely.
     * <p>
     * This will zero the entire odometry, and place the robot at 0,0
     */
    public void tareEverything() {
        for (int i = 0; i < swerve_modules.length; ++i) {
            swerve_modules[i].resetPosition();
            swerve_modules[i].setWheelOffsets();
            io_.module_positions[i] = swerve_modules[i].getPosition(true);
        }
    }

    /**
     * Gets the raw value from the Robot IMU
     */
    public Rotation2d getImuYaw() {
        return io_.robot_yaw_;
    }

    /**
     * Returns the module locations in reference to the center of the robot as an
     * array
     * [FrontLeft, FrontRight, BackLeft, BackRight]
     */
    public SwerveModulePosition[] getModulePositions() {
        return io_.module_positions;
    }

    /**
     * Returns the module states of the swerve drive as an array
     * [FrontLeft, FrontRight, BackLeft, BackRight]
     */
    public SwerveModuleState[] getModuleStates() {
        return io_.current_module_states_;
    }

    public void setTargetRotation(Rotation2d target_angle_) {
        // io_.target_rotation_ = target_angle_.rotateBy(io_.field_relative_offset);
        io_.target_rotation_ = target_angle_;

    }

    /**
     * updates the mode flag thats changes what request is applied to the drive
     * train
     * 
     * @param mode drive to switch to [ROBOT_CENTRIC, FIELD_CENTRIC]
     */
    public void setDriveMode(DriveMode mode) {
        io_.drive_mode_ = mode;
    }

    /**
     * Toggles between field centric and robot centric DriveMode
     */
    public void toggleFieldCentric(){
        if(io_.drive_mode_ == DriveMode.FIELD_CENTRIC){
            io_.drive_mode_ = DriveMode.ROBOT_CENTRIC;
        }
        else{
            io_.drive_mode_ = DriveMode.FIELD_CENTRIC;
        }
    }

    /**
     * Plain-Old-Data class holding the state of the swerve drivetrain.
     * This encapsulates most data that is relevant for telemetry or
     * decision-making from the Swerve Drive.
     */
    public static class SwerveDriverainPeriodicIo implements Logged {
        @Log.File
        public SwerveModuleState[] current_module_states_, requested_module_states_;
        @Log.File
        public SwerveModulePosition[] module_positions;
        @Log.File
        public Rotation2d field_relative_offset_ = new Rotation2d();
        @Log.File
        public Rotation2d robot_yaw_ = new Rotation2d();
        @Log.File
        public double driver_joystick_leftX_ = 0.0;
        @Log.File
        public double driver_joystick_leftY_ = 0.0;
        @Log.File
        public double driver_joystick_rightX_ = 0.0;
        @Log.File
        public ChassisSpeeds chassis_speeds_ = new ChassisSpeeds();
        @Log.File
        public Rotation2d target_rotation_ = new Rotation2d();
        @Log.File
        public DriveMode drive_mode_ = DriveMode.ROBOT_CENTRIC;
    }

    @Override
    public Logged getLogged(){
        return io_;
    }
}
