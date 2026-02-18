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

  public Transfer(TransferIO kickerIO, TransferIO regulatorIO) {
    kicker = kickerIO;
    regulator = regulatorIO;
  }

  @Override
  public void periodic() {

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

  public void setKickerVelocity(double velocity) {
    kicker.setVelocity(velocity);
  }

  public void startTransfer(double shooterSpeed) {
    TeleopCommands.globalState = ShooterState.SPINUP;
  }

  public void stop() {
    kicker.stop();
    regulator.stop();
  }

  public boolean withinSpeed() {
    return MathUtil.isNear(12, regulator.getVelocity(), 1);
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
