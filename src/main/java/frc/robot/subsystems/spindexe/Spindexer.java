package frc.robot.subsystems.spindexe;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.spindexer.SpindexerIOInputsAutoLogged;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {

  private final SpindexerIO kSpindexerHardware;
  private final SpindexerIOInputsAutoLogged kSpindexerInputs = new SpindexerIOInputsAutoLogged();
  private double constantVel = 0.0;

  public Spindexer(SpindexerIO kSpindexerIO) {
    this.kSpindexerHardware = kSpindexerIO;
  }

  @Override
  public void periodic() {
    kSpindexerHardware.updateInputs(kSpindexerInputs);
    Logger.processInputs("Spindexer/Inputs", kSpindexerInputs);

    if (Constants.currentMode == Mode.SIM) {
      constantSetVel(constantVel);
    }
  }

  public void setVelocity(double velocity) {
    constantVel = velocity;
    kSpindexerHardware.setVelocity(velocity);
    if (Constants.currentMode == Mode.SIM && constantVel != 0) {
      constantVel = velocity;
    }
  }

  private void constantSetVel(double velocity) {
    kSpindexerHardware.setVelocity(velocity);
  }

  @AutoLogOutput(key = "Spindexer/Outputs/Velocity")
  public double getVelocity() {
    return kSpindexerHardware.getVelocity();
  }

  public void stopSpindexer() {
    kSpindexerHardware.stop();
    constantVel = 0;
  }
}
