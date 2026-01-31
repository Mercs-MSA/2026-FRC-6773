package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class TeleopCommands {
  private CommandXboxController kController;

  public static Command spin() {
    return Commands.runOnce(null, null);
  }
}
