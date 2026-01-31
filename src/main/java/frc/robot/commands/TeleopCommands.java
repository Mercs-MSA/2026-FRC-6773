package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Spindexer.Spindexer;

public class TeleopCommands {
  private CommandXboxController kController;

  private Spindexer mIndexer;

  public TeleopCommands(Spindexer indexer) {
    mIndexer = indexer;
  }

  public Command spin(double vel) {
    return Commands.runOnce(
        () -> {
          mIndexer.setVelocity(vel);
        },
        mIndexer);
  }

  public Command stop() {
    return Commands.runOnce(
        () -> {
          mIndexer.stopSpindexer();
        },
        mIndexer);
  }
}
