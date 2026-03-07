package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase { // TODO: Tunable Numbers as needed
  public enum IntakeState {
    STOW(() -> Rotation2d.fromRotations(0.0), 0),
    IDLE(() -> Rotation2d.fromRotations(0.0), -1),
    AGITATE(() -> Rotation2d.fromRotations(0.2), -5),
    INTAKING(() -> Rotation2d.fromRotations(0.2), -16);

    private Supplier<Rotation2d> pivotPos;
    private double rollerVol;

    private IntakeState(Supplier<Rotation2d> pivot, double roller) {
      pivotPos = pivot;
      rollerVol = roller;
    }

    public Rotation2d getPivotPos() {
      return pivotPos.get();
    }

    public double getRollerVol() {
      return rollerVol;
    }
  }

  public IntakeState intakeState;

  private Rotation2d pivotGoal;

  private double agitateTimestamp;

  private boolean resetAgitate = true;

  private final IntakeRollerIO rollerHardware;
  private final IntakeRollerIOInputsAutoLogged rollerInputs = new IntakeRollerIOInputsAutoLogged();

  private final IntakePivotIO pivotHardware;
  private final IntakePivotIOInputsAutoLogged pivotInputs = new IntakePivotIOInputsAutoLogged();

  public Intake(IntakeRollerIO rollerIO, IntakePivotIO pivotIO) {
    rollerHardware = rollerIO;
    pivotHardware = pivotIO;
    intakeState = IntakeState.STOW;
  }

  @Override
  public void periodic() {
    rollerHardware.updateInputs(rollerInputs);
    pivotHardware.updateInputs(pivotInputs);
    Logger.processInputs("Intake/Inputs/Roller", rollerInputs);
    Logger.processInputs("Intake/Inputs/Pivot", pivotInputs);

    Logger.recordOutput("Intake/RollerVelocityRotPerSec", rollerInputs.velocityRotPerSec);
    Logger.recordOutput("Intake/PivotVelocityRotPerSec", pivotInputs.velocityRotPerSec);

    switch (intakeState) {
      case STOW:
        resetAgitate = true;
        pivotGoal = intakeState.getPivotPos();
        break;
      case IDLE:
        resetAgitate = true;
        pivotGoal = intakeState.getPivotPos();
        break;
      case AGITATE: // https://www.desmos.com/calculator/ogflv9fvuk agitation visual
        if (resetAgitate) {
          agitateTimestamp = System.currentTimeMillis();
          resetAgitate = false;
        }
        double a = 0.035;
        double lim = intakeState.getPivotPos().getRotations();
        pivotGoal =
            Rotation2d.fromRotations(
                a * Math.cos(4 * Math.PI * (System.currentTimeMillis() - agitateTimestamp))
                    + (lim - a));
        break;
      case INTAKING:
        resetAgitate = true;
        pivotGoal = intakeState.getPivotPos();
        break;
      default:
        resetAgitate = true;
        pivotGoal = IntakeState.IDLE.getPivotPos();
        break;
    }

    setPivotPosition(pivotGoal);
    setRollerVoltage(intakeState.getRollerVol());
  }

  public Command setIntakeStateCommand(IntakeState state) {
    return Commands.runOnce(
        () -> {
          intakeState = state;
        }); // TODO: Make run instead of runOnce?
  }

  public void setIntakeState(IntakeState state) {
    intakeState = state;
  }

  public void setPivotPosition(Rotation2d goal) {
    pivotHardware.setPosition(goal);
  }

  public void setPivotVoltage(double voltage) {
    pivotHardware.setVoltage(voltage);
  }

  public void setRollerVoltage(double voltage) {
    rollerHardware.setVoltage(voltage);
  }

  public void stopRollers() {
    rollerHardware.stop();
  }

  public void stopPivot() {
    pivotHardware.stop();
  }

  public void setPivotBrakeMode(boolean enableBrake) {
    pivotHardware.setBrakeMode(enableBrake);
  }

  // TODO: public void setPivotGains()

  public void setRollerBrakeMode(boolean enableBrake) {
    rollerHardware.setBrakeMode(enableBrake);
  }

  @AutoLogOutput(key = "States/IntakeState")
  public IntakeState getIntakeState() {
    return intakeState;
  }
}
