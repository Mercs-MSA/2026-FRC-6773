package frc.robot.subsystems.transfer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Transfer extends SubsystemBase {
  // Note that regular will be able to set velocity while kicker will only have setVoltage;
  private TransferIOInputsAutoLogged kickerInputs = new TransferIOInputsAutoLogged();
  private TransferIO kicker;
  // Note that regular will be able to set velocity while kicker will only have setVoltage;
  private TransferIOInputsAutoLogged regulatorInputs = new TransferIOInputsAutoLogged();
  private TransferIO regulator;
  
  private double desiredShooterSpeed;

  public enum TransferState {
    STOW,
    SPEEDINGUP,
    READY,
    FREEFORM,
  }

  public TransferState transferState;

  public Transfer(TransferIO kickerIO, TransferIO regulatorIO) {
    transferState = TransferState.STOW;
    kicker = kickerIO;
    regulator = regulatorIO;
  }

  @Override
  public void periodic() {
    switch (transferState) {
      case READY:
        if (!withinSpeed()) {
          kicker.stop();
          transferState = TransferState.SPEEDINGUP;
        }
        break;
      case SPEEDINGUP:
        regulator.setVelocity(desiredSpeed());
        if (withinSpeed()) {
          kicker.setVoltage(TransferConstants.kKickerVoltage.getAsDouble());
        }
        break;
      case STOW:
        regulator.stop();
        kicker.stop();
        break;
      case FREEFORM:
        break;
    }

    kicker.updateInputs(kickerInputs);
    regulator.updateInputs(regulatorInputs);
    Logger.processInputs("Transfer/Inputs/Regulator", regulatorInputs);
    Logger.processInputs("Transfer/Inputs/Kicker", kickerInputs);
  }

  public void setRegulatorVelocity(double velocity) {
    transferState = TransferState.FREEFORM;
    regulator.setVelocity(velocity);
  }

  public void setKickerVoltage(double voltage) {
    transferState = TransferState.FREEFORM;
    kicker.setVoltage(voltage);
  }

  public void startTransfer(double shooterSpeed) {
    desiredShooterSpeed = shooterSpeed;
    transferState = TransferState.SPEEDINGUP;
  }

  public TransferState getState() {
    return this.transferState;
  }

  public void stopTransfer() {
    transferState = TransferState.STOW;
  }

  public boolean withinSpeed() {
    return (regulator.getVelocity()
        > desiredShooterSpeed * TransferConstants.kMinRegulatorVelocityScalar.getAsDouble());
  }

  public double desiredSpeed() {
    return desiredShooterSpeed * TransferConstants.kMinRegulatorVelocityScalar.getAsDouble();
  }

  @AutoLogOutput(key = "Transfer/Regulator/Velocity")
  public double getRegulatorSpeed() {
    return regulator.getVelocity();
  }

  @AutoLogOutput(key = "Transfer/Kicker/Velocity")
  public double getKickerSpeed() {
    return kicker.getVelocity();
  }
}
