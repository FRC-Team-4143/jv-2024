// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberMode;


public class ExtendClimber extends Command {

  static ClimberSubsystem climber_ = ClimberSubsystem.getInstance();

  public ExtendClimber() {
    addRequirements(climber_);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber_.setMode(ClimberMode.EXTEND);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber_.setMode(ClimberMode.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}