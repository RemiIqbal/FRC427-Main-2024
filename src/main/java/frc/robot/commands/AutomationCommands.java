package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToGround;
import frc.robot.subsystems.drivetrain.commands.MoveToSpeaker;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeFromGround;

public class AutomationCommands {
    
  public static Command autoIntakeCommand() {
    // return null; 
    return Commands.sequence(
        new GoToGround(Arm.getInstance()), 
        new IntakeFromGround(Intake.getInstance(), Constants.IntakeConstants.kSuckerManualSpeed)
    ).finallyDo(() -> {
      Arm.getInstance().goToAngle(Constants.ArmConstants.kTravelPosition);
    });
  }

  public static Command pathFindToSpeaker() {
    return Commands.deferredProxy(() -> MoveToSpeaker.goToSpeaker()); 
  }
}
