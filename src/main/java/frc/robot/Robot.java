// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;
import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  // Pivot Constants
  private static final double INTAKE_PIVOT_OUT = Math.toRadians(-30);
  private static final double INTAKE_PIVOT_IN = Math.toRadians(190);
  private static final double INTAKE_PIVOT_OFFSET = 0.29; // Encoder Native Unit (Rotations)
  private static final double INTAKE_PIVOT_CONTROLLER_P = 0.8;
  private static final double INTAKE_PIVOT_CONTROLLER_D = 0.00;
  private static final double INTAKE_PIVOT_CONTROLLER_FF = 0.55;

  double current_pivot_angle = 0.0;
  double intake_pivot_setpoint = INTAKE_PIVOT_IN;

  // Shooter Constants
  public static final double SHOOT_SPEED = 0.99;
  public static final double SHOOT_SPEED_SPIN_FACTOR = 0.8;

  private RobotContainer m_robotContainer;
  // Shooter
  public CANSparkMax upperLeftShooter = new CANSparkMax(3, MotorType.kBrushless);
  public CANSparkMax upperRightShooter = new CANSparkMax(1, MotorType.kBrushless);
  public CANSparkMax lowerLeftShooter = new CANSparkMax(4, MotorType.kBrushless);
  public CANSparkMax lowerRightShooter = new CANSparkMax(2, MotorType.kBrushless);

  // Intake
  public CANSparkMax intakeRollers = new CANSparkMax(7, MotorType.kBrushless);
  private CANSparkMax intakePivot = new CANSparkMax(8, MotorType.kBrushless);
  private SparkAbsoluteEncoder intakePivotEncoder = intakePivot.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private SparkPIDController intakePivotController = intakePivot.getPIDController();

  // Climber
  private CANSparkMax Leftclimber = new CANSparkMax(6, MotorType.kBrushless);
  private CANSparkMax Rightclimber = new CANSparkMax(5, MotorType.kBrushless);
 
  private XboxController driver_controller = new XboxController(0);

  public Command shootAndMoveAuto;
  public Command shootAndDONTMoveAuto;

  public SendableChooser<Command> autoChooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    m_robotContainer = RobotContainer.getInstance();
    OI.configureBindings();
    CameraServer.startAutomaticCapture();
    SmartDashboard.setDefaultNumber("Mode", 0);
    

    Leftclimber.setSmartCurrentLimit(80);
    Rightclimber.setSmartCurrentLimit(80);
    Leftclimber.setIdleMode(IdleMode.kBrake);
    Rightclimber.setIdleMode(IdleMode.kBrake);

    // Encoder Config
    intakePivotController.setFeedbackDevice(intakePivotEncoder);
    intakePivotController.setP(INTAKE_PIVOT_CONTROLLER_P);
    intakePivotController.setD(INTAKE_PIVOT_CONTROLLER_D);
    intakePivot.setInverted(true);
    

    shootAndMoveAuto = Commands.startEnd(
      () -> {lowerRightShooter.set(SHOOT_SPEED);
        upperRightShooter.set(SHOOT_SPEED);
        upperLeftShooter.set(-SHOOT_SPEED*SHOOT_SPEED_SPIN_FACTOR);
        lowerLeftShooter.set(-SHOOT_SPEED*SHOOT_SPEED_SPIN_FACTOR);
        intakeRollers.set(-0.99);}, 
        () -> {lowerLeftShooter.set(0);
        lowerRightShooter.set(0);
        upperLeftShooter.set(0);
        upperRightShooter.set(0);
        intakeRollers.set(0);}).withTimeout(2)
        .andThen(Commands.startEnd(
            () -> {SwerveDrivetrain.getInstance().setDriveMode(DriveMode.AUTONOMOUS);}, 
            () -> {SwerveDrivetrain.getInstance().setDriveMode(DriveMode.ROBOT_CENTRIC);},
            SwerveDrivetrain.getInstance()).withTimeout(2)
          );

    shootAndDONTMoveAuto = Commands.startEnd(
      () -> {lowerRightShooter.set(SHOOT_SPEED);
        upperRightShooter.set(SHOOT_SPEED);
        upperLeftShooter.set(-SHOOT_SPEED*SHOOT_SPEED_SPIN_FACTOR);
        lowerLeftShooter.set(-SHOOT_SPEED*SHOOT_SPEED_SPIN_FACTOR);
        intakeRollers.set(-0.99);}, 
        () -> {lowerLeftShooter.set(0);
        lowerRightShooter.set(0);
        upperLeftShooter.set(0);
        upperRightShooter.set(0);
        intakeRollers.set(0);}).withTimeout(2);

    autoChooser.addOption("Shoot & Go", shootAndMoveAuto);
    autoChooser.addOption("Shoot & Stay", shootAndDONTMoveAuto);
    autoChooser.setDefaultOption("Shoot & Stay", shootAndDONTMoveAuto);

    SmartDashboard.putData("Auto Selector",autoChooser);
  }

  @Override
  public void robotPeriodic() {
    // Call the scheduler so that commands work for buttons
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Pivot Position (rots)", intakePivotEncoder.getPosition());
    SmartDashboard.putNumber("Pivot Power (pct)", intakePivot.getAppliedOutput());

    current_pivot_angle =  (intakePivotEncoder.getPosition() - INTAKE_PIVOT_OFFSET) *  (2*Math.PI);

    SmartDashboard.putNumber("Pivot Position (degs)", Math.toDegrees(current_pivot_angle));


    // tell the subsystems to output telemetry to smartdashboard
    m_robotContainer.outputTelemetry();
  }

  @Override
  public void disabledInit() {
    // m_robotContainer.stopLog();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    // m_robotContainer.initLogfile("AUTO");

    m_autonomousCommand= autoChooser.getSelected();
    m_autonomousCommand.schedule();

  }

  @Override
  public void autonomousExit() {
    SwerveDrivetrain.getInstance().setDriveMode(DriveMode.ROBOT_CENTRIC);
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // m_robotContainer.initLogfile("TELE-OP");

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    SwerveDrivetrain.getInstance().setDriveMode(DriveMode.FIELD_CENTRIC);

  }

  @Override
  public void teleopPeriodic() {
    double mode = SmartDashboard.getNumber("Mode", 0);

    // ----------- Manual Control -----------
    if (driver_controller.getYButton()) {
      Leftclimber.set(-0.5);
      Rightclimber.set(-0.5);
    } else if (driver_controller.getAButton()) {
      Leftclimber.set(0.5);
      Rightclimber.set(0.5);
    } else {
      Leftclimber.set(0);
      Rightclimber.set(0);
    }

    // Pivot Control
    if (driver_controller.getRightBumper()) {
      intake_pivot_setpoint = INTAKE_PIVOT_OUT;
    }
    if (driver_controller.getLeftBumper()) {
      intake_pivot_setpoint = INTAKE_PIVOT_IN;
    }
    SmartDashboard.putNumber("Pivot Setpoint (degs)", Math.toDegrees(intake_pivot_setpoint));

    double pivot_arb_ff = Math.cos(current_pivot_angle) * INTAKE_PIVOT_CONTROLLER_FF;
    SmartDashboard.putNumber("Pivot FF", pivot_arb_ff);
    intakePivotController.setReference(((intake_pivot_setpoint / (2 * Math.PI)) + INTAKE_PIVOT_OFFSET), ControlType.kPosition, 0, pivot_arb_ff);


    // Shooter Control
    if (mode == 0.0) {
      if (driver_controller.getXButton()) {
        lowerRightShooter.set(SHOOT_SPEED);
        upperRightShooter.set(SHOOT_SPEED);
        upperLeftShooter.set(-SHOOT_SPEED*SHOOT_SPEED_SPIN_FACTOR);
        lowerLeftShooter.set(-SHOOT_SPEED*SHOOT_SPEED_SPIN_FACTOR);
      } else {
        lowerLeftShooter.set(0);
        lowerRightShooter.set(0);
        upperLeftShooter.set(0);
        upperRightShooter.set(0);
      }
      if (driver_controller.getLeftTriggerAxis() > 0.1) {
        intakeRollers.set(driver_controller.getLeftTriggerAxis() * .9);
      } else if (driver_controller.getRightTriggerAxis() > 0.1) {
        intakeRollers.set(-driver_controller.getRightTriggerAxis() * .9);
      } else {
        intakeRollers.set(0);
      }
    }

    // 
    if (mode == 1.0) {
      if (driver_controller.getBButton()) {
        lowerRightShooter.set(-0.3);
        upperRightShooter.set(-0.3);
        upperLeftShooter.set(0.3);
        lowerLeftShooter.set(0.3);
        intakeRollers.set(0.15);
      } else {
        lowerRightShooter.set(0);
        upperRightShooter.set(0);
        upperLeftShooter.set(0);
        lowerLeftShooter.set(0);
        intakeRollers.set(0);
      }
    }

    if(driver_controller.getRightStickButtonPressed()){
      SwerveDrivetrain.getInstance().toggleFieldCentric();
    }

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    SwerveDrivetrain.getInstance().seedFieldRelative();
  }

  @Override
  public void testPeriodic() {

  }
}
