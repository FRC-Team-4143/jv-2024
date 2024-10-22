// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.AutoManager;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  public Command shootAndMoveAuto;


  @Override
  public void robotInit() {
    m_robotContainer = RobotContainer.getInstance();
    OI.configureBindings();
    CameraServer.startAutomaticCapture();
    AutoManager.getInstance();
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

    m_autonomousCommand = AutoManager.getInstance().getAutonomousCommand();

    if(m_autonomousCommand != null){
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    SwerveDrivetrain.getInstance().setDriveMode(DriveMode.FIELD_CENTRIC);

  }

  @Override
  public void teleopPeriodic() {

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
