// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriveConstants {
    // Can bus names for each of the swerve modules
    public String[] CANbusName = { "rio", "rio", "rio", "rio" };

    // Can bus ID for the pigeon
    public int Pigeon2Id = 0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {
    public static final int frontLeftEncoder = 0;
    public static final int frontRightEncoder = 1;
    public static final int backLeftEncoder = 2;
    public static final int backRightEncoder = 3;
    public static final double MaxSpeed = 6; // 6 meters per second desired top speed
    public static final double MaxAngularRate = Math.PI * 2; // Half a rotation per second max angular velocity

  }

  public static class PickupConstants{
    // Motor IDs
    public static final int PIVOT_MOTOR_ID = 8;
    public static final int ROLLER_MOTOR_ID = 7;

    public static final boolean PIVOT_MOTOR_INV = true;
    public static final boolean ROLLER_MOTOR_INV = false;

    // Pivot Control
    public static final double PIVOT_OUT = Math.toRadians(-30);
    public static final double PIVOT_IN = Math.toRadians(180);
    public static final double PIVOT_SHOOT = Math.toRadians(190);
    public static final double PIVOT_OFFSET = 0.29; // Encoder Native Unit (Rotations)
    public static final double PIVOT_CONTROLLER_P = 0.8;
    public static final double PIVOT_CONTROLLER_D = 0.00;
    public static final double PIVOT_CONTROLLER_FF = 0.35;

    // Speeds
    public static final double ROLLER_PICKUP_SPEED = 0.75;
    public static final double ROLLER_SHOOT_SPEED = -1.0;
    public static final double ROLLER_INDEX_SPEED = 0.1;
  }

  public static class ShooterConstants{
    // Motor IDs
    public static final int UPPER_LEFT_MOTOR_ID = 3;
    public static final int LOWER_LEFT_MOTOR_ID = 4;
    public static final int UPPER_RIGHT_MOTOR_ID = 1;
    public static final int LOWER_RIGHT_MOTOR_ID = 2;

    // Speeds
    public static final double SHOOT_SPEED = 1.0;
    public static final double SHOOT_SPEED_SPIN_FACTOR = 0.8;
    public static final double SOURCE_INTAKE_SPEED = -0.3;
  }

  public static class ClimberConstants{
    // Motor IDs
    public static final int LEFT_MOTOR_ID = 6;
    public static final int RIGHT_MOTOR_ID = 5;

    public static final boolean LEFT_MOTOR_INV = true;
    public static final boolean RIGHT_MOTOR_INV = true;

    // Speeds
    public static final double EXTEND_SPEED = 0.5;
    public static final double RETRACT_SPEED = -0.5;
  }

}
