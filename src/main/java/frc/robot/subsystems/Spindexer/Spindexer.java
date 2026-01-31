package frc.robot.subsystems.Spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {

  private final SpindexerIO kSpindexerHardware;
  private final SpindexerIOInputsAutoLogged kSpindexerInputs = new SpindexerIOInputsAutoLogged();

  public Spindexer(SpindexerIO kSpindexerIO) {
    this.kSpindexerHardware = kSpindexerIO;
  }

  @Override
  public void periodic() {
    kSpindexerHardware.updateInputs(kSpindexerInputs);
    Logger.processInputs("Spindexer/Inputs", kSpindexerInputs);
  }

  public void setVelocity(double velocity) {
    kSpindexerHardware.setVelocity(velocity);
  }

  @AutoLogOutput(key = "Spindexer/Outputs/Velocity")
  public double getVelocity() {
    return kSpindexerHardware.getVelocity();
  }

  public void stopSpindexer() {
    kSpindexerHardware.stop();
  }
}
