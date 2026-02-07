package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.transfer.Transfer;

public class TeleopCommands {
  private CommandXboxController kController;

  private Spindexer mIndexer;
  private Transfer mTransfer;

  public TeleopCommands(Spindexer indexer, Transfer transfer) {
    mIndexer = indexer;
    mTransfer = transfer;
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

  public Command startTransfer(double shooterSpeed) {
    return Commands.runOnce(
        () -> {
          mTransfer.startTransfer(shooterSpeed);
        },
        mTransfer);
  }

  public Command stopTransfer() {
    return Commands.runOnce(
        () -> {
          mTransfer.stopTransfer();
        },
        mTransfer);
  }

  public Command stopKicker() {
    return Commands.runOnce(
        () -> {
          mTransfer.setKickerVoltage(0);
        },
        mTransfer);
  }
}
