package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.transfer.Transfer;

public class TeleopCommands {
  // private CommandXboxController kController;

  private Transfer transfer;

  public TeleopCommands(Transfer transfer) {
    this.transfer = transfer;
  }

  public Command startTransfer(double shooterSpeed) {
    return Commands.runOnce(
        () -> {
          transfer.startTransfer(shooterSpeed);
        },
        transfer);
  }

  public Command stopTransfer() {
    return Commands.runOnce(
        () -> {
          transfer.stopTransfer();
        },
        transfer);
  }

  public Command stopKicker() {
    return Commands.runOnce(
        () -> {
          transfer.setKickerVoltage(0);
        },
        transfer);
  }
}
