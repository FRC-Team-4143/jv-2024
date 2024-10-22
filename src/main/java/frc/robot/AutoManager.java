package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.commad.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;
import frc.robot.commands.*;



public class AutoManager {

    // Singleton pattern
    private static AutoManager autoManagerInstance = null;

    public static AutoManager getInstance() {
        if (autoManagerInstance == null) {
            autoManagerInstance = new AutoManager();
        }
        return autoManagerInstance;
    }

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    private AutoManager() {

        autoChooser.addOption("Shoot & Go", new Shoot().withTimeout(5).andThen(new AutoReverse().withTimeout(2)));
        autoChooser.addOption("Shoot & Stay", new Shoot().withTimeout(5));
        autoChooser.addOption("Nothing", new WaitCommand(5))

        autoChooser.setDefaultOption("Nothing", new WaitCommand(5));
        SmartDashboard.putData("Selected Auto", autoChooser);
    }

    public SendableChooser<Command> getAutoChooser(){
        return autoChooser;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}