package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.InchesPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  public enum ClimbState {
    STOW,
    AUTON_CLIMB,
    TELEOP_CLIMB
  }

  public ClimbState climbState;

  private final ClimbIO climbHardware;
  private final ClimbIOInputsAutoLogged climbInputs = new ClimbIOInputsAutoLogged();

  public Climb(ClimbIO climbIO) {
    climbHardware = climbIO;
    climbState = ClimbState.STOW;
  }

  @Override
  public void periodic() {
    climbHardware.updateInputs(climbInputs);
    Logger.processInputs("Climb/Inputs", climbInputs);

    switch (climbState) {
      case STOW:
        stop();
        break;
      case TRANSFERRING:
        setTangentialVelocity(Constants.fuelLaunchVelocity);
        break;
    }
  }

  public void setClimbState(ClimbState state) {
    climbState = state;
  }

  public void setVoltage(double voltage) {
    climbHardware.setVoltage(voltage);
  }

  public void stop() {
    climbHardware.stop();
  }

  @AutoLogOutput(key = "States/ClimbState")
  public ClimbState getClimbState() {
    return climbState;
  }
}
