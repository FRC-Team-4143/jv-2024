// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.ClimberConstants;

import monologue.Logged;
import monologue.Annotations.Log;

public class ClimberSubsystem extends Subsystem {

    // Singleton pattern
    private static ClimberSubsystem climber_instance = null;

    public static ClimberSubsystem getInstance() {
        if (climber_instance == null) {
            climber_instance = new ClimberSubsystem();
        }
        return climber_instance;
    }

    /**
     * Class Members
     */
    private ClimberSubsystemPeriodicIo io_;

    public CANSparkMax right_motor_;
    public CANSparkMax left_motor_;

    public enum ClimberMode {
        EXTEND,
        IDLE,
        RETRACT
    }

    private ClimberSubsystem() {
        io_ = new ClimberSubsystemPeriodicIo();
        left_motor_ = new CANSparkMax(ClimberConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        right_motor_ = new CANSparkMax(ClimberConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
        reset();
    }

    @Override
    public void reset() {
        left_motor_.setInverted(ClimberConstants.LEFT_MOTOR_INV);
        left_motor_.setIdleMode(IdleMode.kBrake);
        left_motor_.setSmartCurrentLimit(80);

        right_motor_.setInverted(ClimberConstants.RIGHT_MOTOR_INV);
        right_motor_.setIdleMode(IdleMode.kBrake);
        right_motor_.setSmartCurrentLimit(80);
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
    }

    @Override
    public void updateLogic(double timestamp) {
        switch(io_.mode_){
            EXTEND:
                target_climber_speed_ = ClimberConstants.EXTEND_SPEED;
                break;
            RETRACT:
                target_climber_speed_ = ClimberConstants.RETRACT_SPEED;
                break;
            IDLE:
            default:
                target_climber_speed_ = 0.0;
                break;
        }
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        left_motor_.set(target_climber_speed_);
        right_motor_.set(target_climber_speed_);
    }

    @Override
    public void outputTelemetry(double timestamp) {
    }

    public void setMode(ClimberMode mode){
        io_.mode = mode;
    }

    public class ClimberSubsystemPeriodicIo implements Logged {
        @Log.File
        public double target_climber_speed_ = 0.0;
        @Log.File
        public ClimberMode mode_ = ClimberMode.IDLE;
    }

    @Override
    public Logged getLogged() {
      return io_;
    }
}