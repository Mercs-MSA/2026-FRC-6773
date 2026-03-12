package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  public enum ClimbState {
    STOW,
    AUTON_CLIMB, // TODO: ADD CORRECT L1 Climb
    AUTON_DESCEND,
    TELEOP_CLIMB, // TODO: ADD CORRECT L1 Climb
    TELEOP_ADJUST;
  }

  public ClimbState climbState;

  public DoubleSupplier operatorAdjustment;

  private final ClimbIO climbHardware;
  private final ClimbIOInputsAutoLogged climbInputs = new ClimbIOInputsAutoLogged();

  public Climb(ClimbIO climbIO, DoubleSupplier op) {
    climbHardware = climbIO;
    climbState = ClimbState.STOW;
    operatorAdjustment = op;
  }

  @Override
  public void periodic() {
    climbHardware.updateInputs(climbInputs);
    Logger.processInputs("Climb/Inputs", climbInputs);

    switch (climbState) {
      case AUTON_CLIMB:
        climbHardware.setPosition(ClimbConstants.L1Pos);
        break;
      case AUTON_DESCEND:
        climbHardware.setVoltage(ClimbConstants.descendClimbVoltage);
      case STOW:
        climbHardware.setPosition(ClimbConstants.stowPos);
        break;
      case TELEOP_CLIMB:
        climbHardware.setPosition(ClimbConstants.L1Pos);
        break;
      case TELEOP_ADJUST:
        if (operatorAdjustment.getAsDouble() != 0) {
          climbHardware.setVoltage(operatorAdjustment.getAsDouble());
        } else {
          climbHardware.setBrakeMode(true);
          climbHardware.stop();
        }
      default:
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
