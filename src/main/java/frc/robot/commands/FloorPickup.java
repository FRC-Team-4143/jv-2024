// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.PickupSubsystem.PickupMode;


public class FloorPickup extends Command {

  static PickupSubsystem pickup_ = PickupSubsystem.getInstance();

  public FloorPickup() {
    addRequirements(pickup_);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pickup_.setMode(PickupMode.FLOOR_PICKUP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Commands.startEnd(() -> pickup_.setMode(PickupMode.INDEX), () -> pickup_.setMode(PickupMode.IDLE), pickup_).withTimeout(.75).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}