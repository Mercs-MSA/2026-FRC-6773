package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase { // TODO: Tunable Numbers as needed
  public enum IntakeState {
    STOW(() -> Rotation2d.fromRotations(0.0), 0),
    IDLE(() -> Rotation2d.fromRotations(0.23), 0),
    BUMP(() -> Rotation2d.fromRotations(0.21), -12),
    AGITATE(() -> Rotation2d.fromRotations(0.2), -5),
    INTAKING(() -> Rotation2d.fromRotations(0.23), -6),
    OUTTAKING(() -> Rotation2d.fromRotations(0.23), 7);

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

  private final double HOPPER_RETRACTION_POINT =
      IntakeState.AGITATE.getPivotPos().getRotations(); // rotations

  private final double AGITATE_AMPLITUDE = 0.045; // rotations

  private double LINEAR_RETRACTION_TIME = 0.0; // seconds

  // Piecewise agitation parameters (see Desmos:
  // https://www.desmos.com/calculator/ogflv9fvuk)
  private final LoggedNetworkBoolean usePiecewiseAgitation =
      new LoggedNetworkBoolean("/Intake/UsePiecewiseAgitation", false);

  // This value represents what percent of time the intake will be at the bottom
  // position (0 to 1),
  // the rest of the time it will be going up and down
  private final LoggedNetworkNumber agitateT = new LoggedNetworkNumber("/Intake/AgitateT", 0);
  // This value represents how smooth the transition between the flat portions and
  // the sin portions
  // will be.
  // 0 is no transition, 1 is a very smooth transition.
  // Non-zero values of C will cause the actual value of T to be higher than it is
  // here, higher
  // values = more T
  private final LoggedNetworkNumber agitateC = new LoggedNetworkNumber("/Intake/AgitateC", 0.75);
  // This value is a multiplier to make the overall sin function go faster.
  private final LoggedNetworkNumber agitateFreq =
      new LoggedNetworkNumber("/Intake/AgitateFreq", 2.0);

  private final LoggedNetworkNumber amplitude = new LoggedNetworkNumber("/Intake/Amplitude", 1.0);

  public IntakeState intakeState;

  private Rotation2d pivotGoal;

  private double agitateTimestamp;

  private boolean resetAgitate = true;

  // private Trigger bumpTrigger;

  private final IntakeRollerIO rollerHardware;
  private final IntakeRollerIOInputsAutoLogged rollerInputs = new IntakeRollerIOInputsAutoLogged();

  private final IntakePivotIO pivotHardware;
  private final IntakePivotIOInputsAutoLogged pivotInputs = new IntakePivotIOInputsAutoLogged();

  public boolean wasIntakingBefore = false;

  public Intake(
      IntakeRollerIO rollerIO,
      IntakePivotIO pivotIO,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    rollerHardware = rollerIO;
    pivotHardware = pivotIO;
    intakeState = IntakeState.STOW;

    // bumpTrigger =
    //     ZoneUtil.BUMP_ZONES.willContain(poseSupplier, fieldSpeedsSupplier, Seconds.of(0.3));
    // bumpTrigger.onTrue(
    //     Commands.runOnce(
    //         () -> {
    //           setIntakeState(IntakeState.BUMP);
    //         }));
    // bumpTrigger.onFalse(
    //     Commands.runOnce(
    //         () -> {
    //           IntakeState newState = IntakeState.IDLE;
    //           if (wasIntakingBefore) newState = IntakeState.INTAKING;
    //           setIntakeState(newState);
    //         }));
    // bumpTrigger.debounce(0.5);
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
        wasIntakingBefore = false;
        resetAgitate = true;
        pivotGoal = intakeState.getPivotPos();
        break;
      case IDLE:
        wasIntakingBefore = false;
        resetAgitate = true;
        pivotGoal = intakeState.getPivotPos();
        break;
        // case BUMP:
        //   resetAgitate = true;
        //   pivotGoal = intakeState.getPivotPos();
        //   break;
      case AGITATE: // https://www.desmos.com/calculator/ogflv9fvuk agitation visual
        wasIntakingBefore = false;
        if (resetAgitate) {
          agitateTimestamp = System.currentTimeMillis();
          resetAgitate = false;
        }

        double x = (System.currentTimeMillis() - agitateTimestamp) / 1000;

        double i = HOPPER_RETRACTION_POINT;

        double a = AGITATE_AMPLITUDE;

        double b = intakeState.getPivotPos().getRotations();

        if (x < 0.0) {
          x = 0.0;
        }
        if (LINEAR_RETRACTION_TIME > 0.0 && x <= LINEAR_RETRACTION_TIME) {
          double m = (i - b) / LINEAR_RETRACTION_TIME;
          pivotGoal = Rotation2d.fromRotations(m * x + b);
        } else if (usePiecewiseAgitation.get()) {
          double hVal = piecewiseH(x * agitateFreq.get());
          pivotGoal = Rotation2d.fromRotations(a * (2 * hVal - 1) + (i - a));
        } else {
          pivotGoal =
              Rotation2d.fromRotations(a * Math.cos(agitateFreq.get() * Math.PI * x) + (i - a));
        }

        break;
      case INTAKING:
        wasIntakingBefore = true;
        resetAgitate = true;
        pivotGoal = intakeState.getPivotPos();
        break;
      case OUTTAKING:
        wasIntakingBefore = false;
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

    Logger.recordOutput("agitate timer", (System.currentTimeMillis() - agitateTimestamp) / 1000);
    // Logger.recordOutput("Intake/Position", pivotHardware.getPosition().);
  }

  /** Piecewise smooth agitation waveform H(x) */
  // @AutoLogOutput(key="INTAKE)
  private double piecewiseH(double x) {
    double t = MathUtil.clamp(agitateT.get(), 0.0, 0.999);
    double c = Math.max(agitateC.get(), 0.001);
    double epsilon = c * (1 - t);
    double sinVal = Math.sin(x);
    double z = sinVal * sinVal - t;

    double h;
    if (z <= 0) {
      h = 0;
    } else if (z < epsilon) {
      double u = z / epsilon;
      h = (z / (1 - t)) * (3 * u * u - 2 * u * u * u);
    } else {
      h = z / (1 - t);
    }
    return 1.0 - amplitude.getAsDouble() * h;
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

  @AutoLogOutput(key = "Intake/POSITIONDEGREES")
  public double getPivotPositionDegrees() {
    return pivotHardware.getPosition().abs(Degrees);
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

  public boolean getWasIntake() {
    return wasIntakingBefore;
  }

  @AutoLogOutput(key = "States/IntakeState")
  public IntakeState getIntakeState() {
    return intakeState;
  }
}
