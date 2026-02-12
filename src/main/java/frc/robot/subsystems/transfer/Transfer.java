package frc.robot.subsystems.transfer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TeleopCommands;
import frc.robot.commands.TeleopCommands.ShooterState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Transfer extends SubsystemBase {
  // Note that regular will be able to set velocity while kicker will only have setVoltage;
  private TransferIOInputsAutoLogged kickerInputs = new TransferIOInputsAutoLogged();
  private TransferIO kicker;
  // Note that regular will be able to set velocity while kicker will only have setVoltage;
  private TransferIOInputsAutoLogged regulatorInputs = new TransferIOInputsAutoLogged();
  private TransferIO regulator;

  private double desiredShooterSpeed;

  public ShooterState shooterState = ShooterState.INACTIVE;

  public Transfer(TransferIO kickerIO, TransferIO regulatorIO) {
    kicker = kickerIO;
    regulator = regulatorIO;
  }

  @Override
  public void periodic() {
    switch (TeleopCommands.globalState) {
      case INACTIVE:
        regulator.stop();
        kicker.stop();
        break;

      case SPINUP:
        regulator.setVelocity(12);
        kicker.stop();
        if (withinSpeed()) TeleopCommands.globalState = ShooterState.SCORE;
        break;

      case SCORE:
        regulator.setVelocity(12);
        kicker.setVoltage(TransferConstants.kKickerVoltage.getAsDouble());
        if (!withinSpeed()) TeleopCommands.globalState = ShooterState.SPINUP;
        break;
    }
    shooterState = TeleopCommands.globalState;

    kicker.updateInputs(kickerInputs);
    regulator.updateInputs(regulatorInputs);
    Logger.processInputs("Transfer/Inputs/Regulator", regulatorInputs);
    Logger.processInputs("Transfer/Inputs/Kicker", kickerInputs);
  }

  public void setRegulatorVelocity(double velocity) {
    regulator.setVelocity(velocity);
  }

  public void setRegulatorVoltage(double voltage) {
    regulator.setVoltage(voltage);
  }

  public void setKickerVoltage(double voltage) {
    kicker.setVoltage(voltage);
  }

  public void startTransfer(double shooterSpeed) {
    desiredShooterSpeed = shooterSpeed;
    TeleopCommands.globalState = ShooterState.SPINUP;
  }

  @AutoLogOutput(key = "Transfer/State")
  public ShooterState getState() {
    return this.shooterState;
  }

  public void stop() {
    TeleopCommands.globalState = ShooterState.INACTIVE;
  }

  public boolean withinSpeed() {
    return MathUtil.isNear(12, regulator.getVelocity(), 1);
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
