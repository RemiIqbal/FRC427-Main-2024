package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToAngle;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;
import frc.robot.subsystems.arm.commands.GoToAngle;
import frc.robot.subsystems.arm.commands.SetVelocity;

public class ShootAnywhere extends SequentialCommandGroup {

    private Drivetrain drivetrain;
    private Arm arm;

    // TODO: Make stuff tunable in limelight

    // First, find a way to get distance from speaker to robot
    // Next, find and turn to angle using trig
    // Turn shooter to angle using setpoints and graphed curve
    // Rev to calculated speend
    // Shoot and LEDs
    // Celebrate!!!

    public ShootAnywhere(Drivetrain drivetrain, Arm arm) {
        
        this.drivetrain = drivetrain;
        this.arm = arm;

        addRequirements(drivetrain, arm);
    }

    public Command shootAnywhere() {
        double angleToTurnRobot = drivetrain.getHeading();
        TurnToAngle turnToAngle = new TurnToAngle(drivetrain, angleToTurnRobot);
        double angleToTurnArm = 0.0;
        GoToAngle goToAngle = new GoToAngle(arm, angleToTurnArm);
        double speedToSetArm = 0.0;
        SetVelocity setVelocity = new SetVelocity(arm, speedToSetArm);
        return Commands.sequence(turnToAngle, goToAngle, setVelocity);
    }


}
