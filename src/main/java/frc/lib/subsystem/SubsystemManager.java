package frc.lib.subsystem;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;

import monologue.Annotations.Log;
import monologue.Logged;
import monologue.Monologue;

public abstract class SubsystemManager {

    // Supposedly 1 is a good starting point, but can increase if we have issues
    private static final int START_THREAD_PRIORITY = 99;

    protected ArrayList<Subsystem> subsystems;
    protected Notifier loopThread;

    public class Contain implements Logged {
        @Log.File
        public ArrayList<Logged> subsystems_io = new ArrayList<>();
    }

    protected Contain ios;
    protected boolean log_init = false;

    public SubsystemManager() {
        // Initialize the subsystem list
        subsystems = new ArrayList<>();

        // create the thread to loop the subsystems and mark as daemon thread so
        // the robot program can properly stop
        loopThread = new Notifier(this::doControlLoop);
        // loopThread.setDaemon(true);

        ios = new Contain();

    }

    public void registerSubsystem(Subsystem system) {
        subsystems.add(system);
        ios.subsystems_io.add(system.getLogged());
    }

    private void doControlLoop() {

        // For each subsystem get incoming data
        double timestamp = Timer.getFPGATimestamp();
        for (Subsystem subsystem : subsystems) {
            try {
                subsystem.readPeriodicInputs(timestamp);
            } catch (Exception e) {
                e.printStackTrace();
                DataLogManager.log(subsystem.getClass().getCanonicalName() + "failed to read inputs");
            }

        }

        // Now update the logic for each subsystem to allow I/O to relax
        timestamp = Timer.getFPGATimestamp();
        for (Subsystem subsystem : subsystems) {
            try {
                subsystem.updateLogic(timestamp);
            } catch (Exception e) {
                e.printStackTrace();
                DataLogManager.log(subsystem.getClass().getCanonicalName() + "failed to update logic");
            }

        }

        // Finally write the outputs to the actuators
        timestamp = Timer.getFPGATimestamp();
        for (Subsystem subsystem : subsystems) {
            try {
                subsystem.writePeriodicOutputs(timestamp);
            } catch (Exception e) {
                e.printStackTrace();
                DataLogManager.log(subsystem.getClass().getCanonicalName() + "failed to write outputs");
            }
        }

        if(log_init && DriverStation.isEnabled()) {
            runLog(Timer.getFPGATimestamp());
        }
    }

    protected void runLog(double timestamp) {
        // If it is valid, collect the subsystem I/Os
        ios.subsystems_io.clear();
        for (Subsystem subsystem : subsystems) {
            ios.subsystems_io.add(subsystem.getLogged());
        }

        try {
            Monologue.updateAll();
        } catch (Exception e) {
            DataLogManager.log("Monologue failed to log io");
        }
    }

    /**
     * Completes the subsystem registration process and begins calling each
     * subsystem in a loop
     */
    protected void completeRegistration() {
        loopThread.startPeriodic(.01);
        Monologue.setupMonologue(ios, "robot", true, false);
        DriverStation.startDataLog(DataLogManager.getLog());
        log_init = true;
    }

    /**
     * When ready to put telemetry out to smartdashboard, call this function to
     * trigger each subsystem to publish its held data. This is supposed to be
     * called by robotPeriodic so telemetry is output in any mode.
     */
    public void outputTelemetry() {
        for (Subsystem subsystem : subsystems) {
            subsystem.outputTelemetry(Timer.getFPGATimestamp());
        }
    }

    /**
     * If subsystems all need to be reset before a robot mode change, call this
     * function to cleanly handle resetting them together. If only one subsystem
     * needs to be reset, that can be accessed through the getInstance method.
     */
    public void reset() {
        for (Subsystem subsystem : subsystems) {
            subsystem.reset();
        }
    }

}
