package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.transfer.Transfer;

public class TeleopCommands {
  public enum ShooterState {
    INACTIVE, // This state still keeps the regulator
    SPINUP, // This is the time needed to reach the desired flywheel velocity
    SCORE // This starts the Spindexer and kicker
    // Note that this state encompasses the Spindexer, Transfer and Shooter + constant tracking
  }

  private CommandXboxController controller;
  private Intake intake;
  private Spindexer mIndexer;
  private Transfer mTransfer;

  public TeleopCommands(
      Intake intake, Spindexer indexer, Transfer transfer, CommandXboxController controller) {
    this.intake = intake;
    this.controller = controller;
    mIndexer = indexer;
    mTransfer = transfer;
    // kClimb = climb;
  }

  public Command runIntakeFloorPickup() {
    return Commands.runOnce(
        () -> {
          intake.setPivotState(IntakeState.kFloorPickup);
          intake.runRollers();
          intake.setBrakeMode(false);
        },
        intake);
  }

  public Command runIntakeStow() {
    return Commands.runOnce(
        () -> {
          intake.setPivotState(IntakeState.kStow);
          intake.stowRollers();
          intake.setBrakeMode(true);
        },
        intake);
  }

  public Command startShooting() {
    return Commands.runOnce(
        () -> {
          mTransfer.startTransfer(12);
        });
  }

  public Command whileShooting() {
    return Commands.run(
        () -> {
          mIndexer.setState(mTransfer.getState());
        });
  }

  public Command stopShooting() {
    return Commands.runOnce(
        () -> {
          mTransfer.stopTransfer();
          mIndexer.setState(ShooterState.INACTIVE);
        });
  }

  public Command spin(double vel) {
    return Commands.runOnce(
        () -> {
          mIndexer.setVelocity(vel);
        },
        mIndexer);
  }

  public Command stop() {
    return Commands.runOnce(
        () -> {
          mIndexer.stopSpindexer();
        },
        mIndexer);
  }

  public Command startTransfer(double shooterSpeed) {
    return Commands.runOnce(
        () -> {
          mTransfer.startTransfer(shooterSpeed);
        },
        mTransfer);
  }

  public Command stopTransfer() {
    return Commands.runOnce(
        () -> {
          mTransfer.stopTransfer();
          mIndexer.setState(ShooterState.INACTIVE);
        },
        mTransfer);
  }

  public Command stopKicker() {
    return Commands.runOnce(
        () -> {
          mTransfer.setKickerVoltage(0);
        },
        mTransfer);
  }
}
