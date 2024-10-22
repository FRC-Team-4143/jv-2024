// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;
import frc.robot.subsystems.PickupSubsystem.PickupMode;


public class SourcePickup extends Command {

  static PickupSubsystem pickup_ = PickupSubsystem.getInstance();
  static ShooterSubsystem shooter_ = ShooterSubsystem.getInstance();

  public SourcePickup() {
    addRequirements(shooter_);
    addRequirements(pickup_);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter_.setMode(ShooterMode.SOURCE_PICKUP);
    pickup_.setMode(PickupMode.SOURCE_PICKUP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter_.setMode(ShooterMode.IDLE);
    pickup_.setMode(PickupMode.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}