package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

public class DriverCommands {


  public Command vibrateController(CommandGenericHID controller, double timeSeconds) {
    return Commands.runOnce(() -> {
      controller.getHID().setRumble(RumbleType.kBothRumble, 1);
    }).andThen(new WaitCommand(timeSeconds)).andThen(() -> { 
        controller.getHID().setRumble(RumbleType.kBothRumble, 0);
    });  
  }
}
