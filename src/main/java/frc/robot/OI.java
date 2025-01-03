// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;



public abstract class OI {

    // Sets up both controllers
    static CommandXboxController driver_joystick_ = new CommandXboxController(0);
    static SwerveDrivetrain swerve_drivetrain_ = SwerveDrivetrain.getInstance();
    // static CommandXboxController operator_joystick_ = new CommandXboxController(1)

    /*
     * Use this method to define controller bindings to commands and other actions
     */
    public static void configureBindings() {
        SmartDashboard.putData("Set Wheel Offsets", Commands.runOnce(
            () -> swerve_drivetrain_.tareEverything())
            .ignoringDisable(true));
    SmartDashboard.putData("Seed Field Centric", Commands.runOnce(
            () -> swerve_drivetrain_.seedFieldRelative())
            .ignoringDisable(true));
    }

    static public double getDriverJoystickLeftX() {
        double val = driver_joystick_.getLeftX();
        double output = val * val;
        output = Math.copySign(output, val);
        return output;
    }

    static public double getDriverJoystickLeftY() {
        double val = driver_joystick_.getLeftY();
        double output = val * val;
        output = Math.copySign(output, val);
        return output;
    }

    static public double getDriverJoystickRightX() {
        double val = driver_joystick_.getRightX();
        double output = val * val;
        output = Math.copySign(output, val);
        return output;
    }
}
