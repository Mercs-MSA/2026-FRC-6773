package frc.robot.subsystems.transfer;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Transfer extends SubsystemBase {
  public enum TransferState {
    IDLE,
    TRANSFERRING
  }

  public TransferState transferState;

  private final TransferIO transferHardware;
  private final TransferIOInputsAutoLogged transferInputs = new TransferIOInputsAutoLogged();

  public Transfer(TransferIO transferIO) {
    transferHardware = transferIO;
    transferState = TransferState.IDLE;
  }

  @Override
  public void periodic() {
    transferHardware.updateInputs(transferInputs);
    Logger.processInputs("Transfer/Inputs", transferInputs);

    switch (transferState) {
      case IDLE:
        stopTransfer();
        break;
      case TRANSFERRING:
        setTangentialVelocity(Constants.fuelLaunchVelocity);
        break;
    }
  }

  public Command setTransferState(TransferState state) {
    return Commands.runOnce(
        () -> {
          transferState = state;
        }); // TODO: Make run instead of runOnce?
  }

  public void setVoltage(double voltage) {
    transferHardware.setVoltage(voltage);
  }

  public void setAngularVelocity(AngularVelocity velocity) {
    transferHardware.setAngularVelocity(velocity);
  }

  public void setTangentialVelocity(LinearVelocity velocity) {
    transferHardware.setTangentialVelocity(velocity);
  }

  public void stopTransfer() {
    transferHardware.stop();
  }
}
