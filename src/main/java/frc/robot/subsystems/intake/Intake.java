// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.util.debugging.LoggedTunableNumber;
import frc.robot.util.visualizers.PivotVisualizer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase {
  public enum IntakeState {
    kFloorPickup(() -> Rotation2d.fromRotations(0.0)),
    kStow(() -> Rotation2d.fromRotations(-0.17)),
    /** Custom setpoint that can be modified over network tables; Useful for debugging */
    custom(
        () ->
            Rotation2d.fromDegrees(
                new LoggedNetworkNumber("Intake/Feedback/PivotSetpointDegrees", 0.0).get()));

    private Supplier<Rotation2d> goalPosition;

    IntakeState(Supplier<Rotation2d> goalPosition) {
      this.goalPosition = goalPosition;
    }

    public Rotation2d getGoalPosition() {
      return this.goalPosition.get();
    }
  }

  private final IntakePivotIO kPivotHardware;
  private final IntakePivotIOInputsAutoLogged kPivotInputs = new IntakePivotIOInputsAutoLogged();

  private final IntakeRollerIO kRollerHardware;
  private final IntakeRollerIOInputsAutoLogged kRollerInputs = new IntakeRollerIOInputsAutoLogged();

  private final LoggedNetworkNumber kP =
      new LoggedNetworkNumber("Intake/Gains/Pivot_kP", IntakeConstants.kPivotGains.p());
  private final LoggedNetworkNumber kI =
      new LoggedNetworkNumber("Intake/Gains/Pivot_kI", IntakeConstants.kPivotGains.i());
  private final LoggedNetworkNumber kD =
      new LoggedNetworkNumber("Intake/Gains/Pivot_kD", IntakeConstants.kPivotGains.d());
  private final LoggedNetworkNumber kS =
      new LoggedNetworkNumber("Intake/Gains/Pivot_kS", IntakeConstants.kPivotGains.s());
  private final LoggedNetworkNumber kV =
      new LoggedNetworkNumber("Intake/Gains/Pivot_kV", IntakeConstants.kPivotGains.v());
  private final LoggedNetworkNumber kA =
      new LoggedNetworkNumber("Intake/Gains/Pivot_kA", IntakeConstants.kPivotGains.a());
  private final LoggedNetworkNumber kG =
      new LoggedNetworkNumber("Intake/Gains/Pivot_kG", IntakeConstants.kPivotGains.g());
  private final LoggedNetworkNumber kMaxVelocity =
      new LoggedNetworkNumber(
          "Intake/MotionMagic/Pivot_kMaxVelocity",
          IntakeConstants.kPivotGains.maxVelocityRotationsPerSecond());
  private final LoggedNetworkNumber kMaxAcceleration =
      new LoggedNetworkNumber(
          "Intake/MotionMagic/Pivot_kMaxAcceleration",
          IntakeConstants.kPivotGains.maxAccelerationRotationsPerSecondSquared());

  // private boolean detectedGamepiece = false;
  private IntakeState pivotState;

  private final PivotVisualizer kPivotVisualizer;
  // TODO add roller visualizer

  /** Creates a new Intake. */
  public Intake(IntakePivotIO pivotHardwareIO, IntakeRollerIO rollerHardwareIO) {
    kPivotHardware = pivotHardwareIO;
    kRollerHardware = rollerHardwareIO;

    kPivotVisualizer =
        new PivotVisualizer(
            "Intake/PivotVisualizer",
            IntakeConstants.kPivotVisualizerConfiguration,
            4.0,
            new Color8Bit(Color.kBlue));

    // TODO: roller visualizer
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    kPivotHardware.updateInputs(kPivotInputs);
    kRollerHardware.updateInputs(kRollerInputs);
    Logger.processInputs("Intake/Inputs/Pivot", kPivotInputs);
    Logger.processInputs("Intake/Inputs/Roller", kRollerInputs);

    if (pivotState != null) {
      setPivotPosition(pivotState.getGoalPosition());
      Logger.recordOutput("Intake/PivotGoalValue", pivotState.getGoalPosition());
      Logger.recordOutput("Intake/PivotGoal", pivotState);
    } else {
      Logger.recordOutput("Intake/PivotGoal", "NONE");
    }

    // Check if pivot is attempting to move beyond its limitations
    if (getPivotPosition().getDegrees() > IntakeConstants.kMaxPivotPosition.getDegrees()
        && kPivotInputs.appliedVoltage > 0.0) {
      stop(false, true);
    } else if (getPivotPosition().getDegrees() < IntakeConstants.kMinPivotPosition.getDegrees()
        && kPivotInputs.appliedVoltage < 0.0) {
      stop(false, true);
    } else {
      // Do nothing if limits are not reached
    }

    // This says that if the value is changed in the advantageScope tool,
    // Then we change the values in the code. Saves deploy time.
    // More found in prerequisites slide
    // LoggedNetworkNumber.ifChanged(
    //   hashCode(),
    //   () -> {
    //     kPivotHardware.setGains(
    //         kP.get(), kI.get(), kD.get(), kS.get(), kG.get(), kV.get(), kA.get());
    //     //TODO: tunable voltage
    //   },
    //   kP,
    //   kI,
    //   kD,
    //   kS,
    //   kV,
    //   kA,
    //   kG);
    // LoggedNetworkNumber.ifChanged(
    //     hashCode(),
    //     () -> {
    //       kPivotHardware.setMotionMagicConstraints(kMaxVelocity.get(), kMaxAcceleration.get());
    //     },
    //     kMaxVelocity,
    //     kMaxAcceleration);

    // The visualizer needs to be periodically fed the current position of the mechanism
    kPivotVisualizer.updatePosition(getPivotPosition().times(-1.0));
  }

  public void setPivotState(IntakeState desiredGoal) {
    pivotState = desiredGoal;
  }

  public boolean ifStowed() {
    if (pivotState.equals(IntakeState.kStow)) {
      return true;
    }
    return false;
  }

  public void stop(boolean stopRollers, boolean stopPivot) {
    if (stopRollers) {
      kRollerHardware.stop();
    }
    if (stopPivot) {
      pivotState = null;
      kPivotHardware.stop();
    }
  }

  public void setPivotVoltage(double voltage) {
    kPivotHardware.setVoltage(voltage);
  }

  public void setPivotPosition(Rotation2d position) {
    kPivotHardware.setPosition(position);
  }

  @AutoLogOutput(key = "Pivot/Feedback/ErrorDegrees")
  public double getPivotErrorDegrees() {
    if (pivotState != null && getPivotPosition() != null) {
      return pivotState.getGoalPosition().getDegrees() - getPivotPosition().getDegrees();
    } else {
      return 0.0;
    }
  }

  public void setBrakeMode(Boolean value) {
    kPivotHardware.setBrakeMode(value);
  }

  @AutoLogOutput(key = "Pivot/Feedback/AtGoal")
  public boolean pivotAtGoal() {
    return Math.abs(getPivotErrorDegrees()) < IntakeConstants.kPivotPositionTolerance.getDegrees();
  }

  public Rotation2d getPivotPosition() {
    return kPivotInputs.position;
  }

  public void runRollers() {
    kRollerHardware.setVoltage(IntakeConstants.kRollerIntakingVoltage);
  }

  public void stopRollers() {
    kRollerHardware.stop();
  }

  public void stowRollers() {
    kRollerHardware.setVoltage(IntakeConstants.kRollerStowVoltage);
  }

  // public void substationIntakeCommand(CommandXboxController controller) {
  //   if (getCoralDetected()) {
  //     setPivotGoal(IntakePivotGoal.kStow);
  //     controller.getHID().setRumble(RumbleType.kBothRumble, 0.5);
  //   } else {
  //     setPivotGoal(IntakePivotGoal.kSubstationPickup);
  //     controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
  //   }
  // }

  public void setRollerVoltage(double volts) {
    kRollerHardware.setVoltage(volts);
  }

  // public boolean getCoralDetected(){
  //   return kRollerInputs.beambreakBroken;
  // }

}
