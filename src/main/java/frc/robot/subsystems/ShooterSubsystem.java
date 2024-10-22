// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.ShooterConstants;

import monologue.Logged;
import monologue.Annotations.Log;

public class ShooterSubsystem extends Subsystem {

    // Singleton pattern
    private static ShooterSubsystem shooter_instance = null;

    public static ShooterSubsystem getInstance() {
        if (shooter_instance == null) {
            shooter_instance = new ShooterSubsystem();
        }
        return shooter_instance;
    }

    /**
     * Class Members
     */
    private ShooterSubsystemPeriodicIo io_;

    public CANSparkMax upper_left_motor_;
    public CANSparkMax lower_left_motor_;
    public CANSparkMax upper_right_motor_;
    public CANSparkMax lower_right_motor_;

    public enum ShooterMode {
        SOURCE_PICKUP,
        IDLE,
        SHOOT
    }

    private ShooterSubsystem() {
        io_ = new ShooterSubsystemPeriodicIo();
        upper_left_motor_ = new CANSparkMax(ShooterConstants.UPPER_LEFT_MOTOR_ID, MotorType.kBrushless);
        lower_left_motor_ = new CANSparkMax(ShooterConstants.LOWER_LEFT_MOTOR_ID, MotorType.kBrushless);
        upper_right_motor_ = new CANSparkMax(ShooterConstants.UPPER_RIGHT_MOTOR_ID, MotorType.kBrushless);
        lower_right_motor_ = new CANSparkMax(ShooterConstants.LOWER_RIGHT_MOTOR_ID, MotorType.kBrushless);
        reset();
    }

    @Override
    public void reset() {
        upper_left_motor_.follow(lower_left_motor_, false);
        upper_right_motor_.follow(lower_right_motor_, false);
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
    }

    @Override
    public void updateLogic(double timestamp) {
        switch(io_.mode_){
            SOURCE_PICKUP:
                target_left_shooter_speed_ = SOURCE_INTAKE_SPEED;
                target_right_shooter_speed_ = SOURCE_INTAKE_SPEED;
                break;
            SHOOT:
                target_left_shooter_speed_ = -SHOOT_SPEED * SHOOT_SPEED_SPIN_FACTOR;
                target_right_shooter_speed_ = SHOOT_SPEED;
                break;
            IDLE:
            default:
                target_left_shooter_speed_ = 0.0;
                target_right_shooter_speed_ = 0.0;
                break;
        }
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        lower_left_motor_.set(target_left_shooter_speed_);
        lower_right_motor_.set(target_right_shooter_speed_);
    }

    @Override
    public void outputTelemetry(double timestamp) {
    }

    public void setMode(ShooterMode mode){
        io_.mode = mode;
    }

    public class ShooterSubsystemPeriodicIo implements Logged {
        @Log.File
        public double target_left_shooter_speed_ = 0.0;
        @Log.File
        public double target_right_shooter_speed_ = 0.0;
        @Log.File
        public ShooterMode mode_ = ShooterMode.IDLE;
    }

    @Override
    public Logged getLogged() {
      return io_;
    }
}