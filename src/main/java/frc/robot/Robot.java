// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;

import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private int counter = 0;
  private double init_rotation;
  private double encoder_diff;

  private RobotContainer m_robotContainer;
  private final Timer m_timer = new Timer();
  private final CANSparkMax upperLeftShooter = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax upperRightShooter = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax lowerLeftShooter = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax lowerRightShooter = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax intakeRollers = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax intakePivot = new CANSparkMax(8, MotorType.kBrushless);
  private final CANSparkMax Leftclimber = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax Rightclimber = new CANSparkMax(5, MotorType.kBrushless);
  private RelativeEncoder m_encoder = intakePivot.getEncoder();

  private final XboxController m_controller = new XboxController(0);

  @Override
  public void robotInit() {
    m_robotContainer = RobotContainer.getInstance();
    AutoManager.getInstance();
    OI.configureBindings();

    init_rotation = m_encoder.getPosition();

    Leftclimber.setSmartCurrentLimit(80);    
    Rightclimber.setSmartCurrentLimit(80);
    Leftclimber.setIdleMode(IdleMode.kBrake);
    Rightclimber.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void robotPeriodic() {
    // Call the scheduler so that commands work for buttons
    CommandScheduler.getInstance().run();

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

    SwerveDrivetrain.getInstance().setDriveMode(DriveMode.AUTONOMOUS);

    m_autonomousCommand = AutoManager.getInstance().getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
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

    encoder_diff = SmartDashboard.getNumber("Encoder Diff:", -45.0);
    SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());

    if (m_controller.getYButton()) {
      Leftclimber.set(0.4);
      Rightclimber.set(-0.4);
    } else if (m_controller.getAButton()) {
      Leftclimber.set(-0.4);
      Rightclimber.set(0.4);
    } else {
      Leftclimber.set(0);
      Rightclimber.set(0);
    }


    if (m_controller.getLeftBumper()) {
      
        intakePivot.set(0.2); 
    
    } else if (m_controller.getRightBumper()) {
        if(m_encoder.getPosition() - init_rotation > encoder_diff) {
          SmartDashboard.putNumber("Position Diff", m_encoder.getPosition() - init_rotation);
          intakePivot.set(-0.2);
        }
    } else {
      intakePivot.set(0);
    }

    if (m_controller.getXButton()) {
      lowerRightShooter.set(0.99);
      upperRightShooter.set(0.99);
      upperLeftShooter.set(-0.8);
      lowerLeftShooter.set(-0.8);
    } else {
    lowerRightShooter.set(0);
      upperRightShooter.set(0);
      upperLeftShooter.set(0);
      lowerLeftShooter.set(0);
    }
    
    if (m_controller.getLeftTriggerAxis() > 0.1) {
      intakeRollers.set(m_controller.getLeftTriggerAxis()*.9);
      counter = 19;
    } else if (m_controller.getRightTriggerAxis() > 0.1) {
      intakeRollers.set(-m_controller.getRightTriggerAxis()*.9);
      counter = 19;
    } else if (m_controller.getBButtonPressed()) {
      intakeRollers.set(0.5);
      counter = 0;
    }
    if (counter > 19){
      intakeRollers.set(0);
    }
    counter++;
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
