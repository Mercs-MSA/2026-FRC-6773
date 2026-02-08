package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.commands.TeleopCommands.ShooterState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {

  private final SpindexerIO kSpindexerHardware;
  private final SpindexerIOInputsAutoLogged kSpindexerInputs = new SpindexerIOInputsAutoLogged();
  private double constantVel = 0.0;

  public ShooterState shooterState = ShooterState.INACTIVE;

  public Spindexer(SpindexerIO kSpindexerIO) {
    this.kSpindexerHardware = kSpindexerIO;
  }

  @Override
  public void periodic() {
    switch (shooterState) {
      case INACTIVE:
        stopSpindexer();
        break;
      case SPINUP:
        stopSpindexer();
      case SCORE:
        setVelocity(12.0);
        break;
    }

    kSpindexerHardware.updateInputs(kSpindexerInputs);
    Logger.processInputs("Spindexer/Inputs", kSpindexerInputs);
  }

  public void setVelocity(double velocity) {
    constantVel = velocity;
    kSpindexerHardware.setVelocity(-1 * velocity);
    if (Constants.currentMode == Mode.SIM && constantVel != 0) {
      constantVel = velocity;
    }
  }

  @AutoLogOutput(key = "Spindexer/Outputs/Velocity")
  public double getVelocity() {
    return kSpindexerHardware.getVelocity();
  }

  @AutoLogOutput(key = "Spindexer/Outputs/StateVal")
  public ShooterState getState() {
    return this.shooterState;
  }

  public void stopSpindexer() {
    kSpindexerHardware.stop();
    constantVel = 0;
  }

  public void setState(ShooterState state) {
    shooterState = state;
  }
}
