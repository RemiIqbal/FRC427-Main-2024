package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Intake;

public class OuttakeToSpeaker extends SequentialCommandGroup {
     // declare how long to intake for and speed
    Intake m_intake;
    double m_speed;

     // uses other commands to start the shooter first (cause its slow) then makes the sucker move a ring to the shooter after waiting a sec
    public OuttakeToSpeaker(Intake intake, double shooterSpeed, double suckSpeed) {
        addCommands(
            new SetShooterSpeed(intake, shooterSpeed), 
            new WaitCommand(1), 
            new SetSuckerIntakeSpeed(intake, suckSpeed),
            new WaitCommand(1)
        );

        finallyDo(() -> {
            intake.stopShoot();
            intake.stopSuck();
        }); 

        addRequirements(intake);
    }
}
