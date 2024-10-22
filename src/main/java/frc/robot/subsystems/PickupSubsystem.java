// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.PickupConstants;

import monologue.Logged;
import monologue.Annotations.Log;

public class PickupSubsystem extends Subsystem {

    // Singleton pattern
    private static PickupSubsystem pickup_instance = null;

    public static PickupSubsystem getInstance() {
        if (pickup_instance == null) {
            pickup_instance = new PickupSubsystem();
        }
        return pickup_instance;
    }

    /**
     * Class Members
     */
    private PickupSubsystemPeriodicIo io_;

    private SparkAbsoluteEncoder pivot_encoder_;
    private CANSparkMax pivot_motor_;
    private CANSparkMax roller_motor_;
    private SparkPIDController pivot_controller_;

    public enum PickupMode {
        FLOOR_PICKUP,
        SOURCE_PICKUP,
        IDLE,
        SHOOT
    }

    private PickupSubsystem() {
        io_ = new PickupSubsystemPeriodicIo();
        pivot_motor_ = new CANSparkMax(PickupConstants.PIVOT_MOTOR_ID , MotorType.kBrushless);
        pivot_encoder_ = pivot_motor_.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)
        roller_motor_ = new CANSparkMax(PickupConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
        pivot_controller_ = pivot_motor_.getPIDController();

        reset();
    }

    @Override
    public void reset() {
        // Encoder Config
        pivot_controller_.setFeedbackDevice(pivot_encoder_);
        pivot_controller_.setP(PickupConstants.PIVOT_CONTROLLER_P);
        pivot_controller_.setD(PickupConstants.PIVOT_CONTROLLER_D);

        pivot_motor_.setInverted(PickupConstants.PIVOT_MOTOR_INV);
        roller_motor_.setInverted(PickupConstants.ROLLER_MOTOR_INV);
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        io_.current_pivot_angle =  (pivot_encoder_.getPosition() - PickupConstants.PIVOT_OFFSET) *  (2*Math.PI);
    }

    @Override
    public void updateLogic(double timestamp) {
        io_.pivot_arb_ff_ = Math.cos(io_.current_pivot_angle) * PickupConstants.PIVOT_CONTROLLER_FF;

        switch(io_.mode_){
            FLOOR_PICKUP:
                io_.target_pivot_angle_ = PickupConstants.PIVOT_OUT;
                io_.target_roller_speed_ = PickupConstants.ROLLER_PICKUP_SPEED;
                break;
            SOURCE_PICKUP:
                io_.target_pivot_angle_ = PickupConstants.PIVOT_IN;
                io_.target_roller_speed_ = PickupConstants.ROLLER_PICKUP_SPEED;
                break;
            SHOOT:
                io_.target_pivot_angle_ = PickupConstants.PIVOT_IN;
                io_.target_roller_speed_ = ;
                break;
            IDLE:
            default:
                io_.target_pivot_angle_ = PickupConstants.PIVOT_IN;
                io_.target_roller_speed_ = 0;
                break;
        }
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        pivot_controller_.setReference(((io_.target_pivot_angle_ / (2 * Math.PI)) + PickupConstants.PIVOT_OFFSET), ControlType.kPosition, 0, io_.pivot_arb_ff_);
        roller_motor_.set(target_roller_speed_);
    }

    @Override
    public void outputTelemetry(double timestamp) {
    }

    public void setMode(PickupMode mode){
        io_.mode = mode_
    }

    public class PickupSubsystemPeriodicIo implements Logged {
        @Log.File
        public double current_pivot_angle_ = 0.0;
        @Log.File
        public double target_pivot_angle_ = 0.0;
        @Log.File
        public double pivot_arb_ff_ = 0.0;
        @Log.File
        public PickupMode mode_ = PickupMode.IDLE;
    }

    @Override
    public Logged getLogged() {
      return io_;
    }
}