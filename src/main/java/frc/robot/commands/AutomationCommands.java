package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToGround;
import frc.robot.subsystems.arm.commands.GoToSpeaker;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.MoveToAmp;
import frc.robot.subsystems.drivetrain.commands.MoveToSpeaker;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeFromGround;
import frc.robot.subsystems.intake.commands.OuttakeToSpeaker;

public class AutomationCommands {
    
  public static Command autoIntakeCommand() {
    return Commands.sequence(
        new GoToGround(Arm.getInstance()), 
        new IntakeFromGround(Intake.getInstance())
    ).finallyDo(() -> {
      Arm.getInstance().goToAngle(Constants.ArmConstants.kTravelPosition);
    });
  }

  public static Command pathFindToSpeaker() {
    return Commands.deferredProxy(() -> MoveToSpeaker.goToSpeaker()); 
  }

  public static Command pathFindToAmp() {
    return Commands.deferredProxy(() -> MoveToAmp.goToAmp()); 
  }

  public static Command pathFindToSpeakerAndScore() {
    // TODO
    return Commands.none(); 
  }

  public static Command pathFindToAmpAndScore() {
    // TODO
    return Commands.none(); 
  }

  public static Command shootFromAnywhere() {
    return Commands.deferredProxy(() -> ShootAnywhere.shootAnywhere(Drivetrain.getInstance(), Arm.getInstance(), Intake.getInstance())); 
  }
}
