package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {

  public enum SpindexerState {
    INACTIVE,
    ACTIVE,
    MANUAL
  }

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
    kSpindexerHardware.setVelocity(-1 * velocity);
  }

  public void setVoltage(double voltage) {
    kSpindexerHardware.setVoltage(voltage);
  }

  @AutoLogOutput(key = "Spindexer/Outputs/Velocity")
  public double getVelocity() {
    return kSpindexerHardware.getVelocity();
  }

  // @AutoLogOutput(key = "Spindexer/Outputs/StateVal")
  // public ShooterState getState() {
  //   return this.shooterState;
  // }

  public void stopSpindexer() {
    kSpindexerHardware.stop();
  }

  // public void setState(ShooterState state) {
  //   shooterState = state;
  // }
}
