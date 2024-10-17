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
import frc.lib.swerve.SwerveRequest;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;
import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private int counter = 0;
  private double init_rotation;
  private double encoder_diff;

  private static final double INTAKE_PIVOT_OUT = 0.23;
  private static final double INTAKE_PIVOT_IN = 0.85;
  private static final double INTAKE_PIVOT_CONTROLLER_P = 0.05;
  private static final double INTAKE_PIVOT_CONTROLLER_D = 0.00;
  public static final double SHOOT_SPEED = 0.99;
  public static final double SHOOT_SPEED_SPIN = 0.8;

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
    intakePivotController.setOutputRange(-0.3, 0.3, 0);
    

    shootAndMoveAuto = Commands.startEnd(
      () -> {lowerRightShooter.set(SHOOT_SPEED);
        upperRightShooter.set(SHOOT_SPEED);
        upperLeftShooter.set(-SHOOT_SPEED*SHOOT_SPEED_SPIN);
        lowerLeftShooter.set(-SHOOT_SPEED*SHOOT_SPEED_SPIN);
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
        upperLeftShooter.set(-SHOOT_SPEED*SHOOT_SPEED_SPIN);
        lowerLeftShooter.set(-SHOOT_SPEED*SHOOT_SPEED_SPIN);
        intakeRollers.set(-0.99);}, 
        () -> {lowerLeftShooter.set(0);
        lowerRightShooter.set(0);
        upperLeftShooter.set(0);
        upperRightShooter.set(0);
        intakeRollers.set(0);}).withTimeout(2);

    autoChooser.addOption("shootAndMoveAuto", shootAndMoveAuto);
    autoChooser.addOption("shootAndDONTMoveAuto", shootAndDONTMoveAuto);
    autoChooser.setDefaultOption("shootAndDONTMoveAuto", shootAndDONTMoveAuto);

    SmartDashboard.putData("Auto",autoChooser);
  }

  @Override
  public void robotPeriodic() {
    // Call the scheduler so that commands work for buttons
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Intake Encoder", intakePivotEncoder.getPosition());

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

    autoChooser.getSelected().schedule();

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
    encoder_diff = SmartDashboard.getNumber("Encoder Diff:", -45.0);
    SmartDashboard.putNumber("Encoder Position", intakePivotEncoder.getPosition());

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

    // 
    if (driver_controller.getLeftBumper()) {
      intakePivotController.setReference(INTAKE_PIVOT_OUT, ControlType.kVoltage);
    }
    if (driver_controller.getRightBumper()) {
      intakePivotController.setReference(INTAKE_PIVOT_IN, ControlType.kVoltage);
    }

    // 
    if (mode == 0.0) {
      if (driver_controller.getXButton()) {
        lowerRightShooter.set(SHOOT_SPEED);
        upperRightShooter.set(SHOOT_SPEED);
        upperLeftShooter.set(-SHOOT_SPEED*SHOOT_SPEED_SPIN);
        lowerLeftShooter.set(-SHOOT_SPEED*SHOOT_SPEED_SPIN);
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
