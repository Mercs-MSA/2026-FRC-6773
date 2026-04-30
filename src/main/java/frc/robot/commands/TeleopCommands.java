package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.Transfer.TransferState;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class TeleopCommands {
  private Intake intake;
  private Indexer indexer;
  private Transfer transfer;
  private Shooter shooter;
  // private Climb climber;

  // private RobotManager manager;

  boolean fixedShooting = false;

  Supplier<IntakeState> intakeStateSupplier;

  Trigger flywheelRamp;

  public TeleopCommands(
      Drive drive,
      Intake intake,
      Indexer indexer,
      Transfer transfer,
      Shooter shooter,
      BooleanSupplier shootSupply
      // , Climb climb
      ) {

    this.intake = intake;
    this.indexer = indexer;
    this.transfer = transfer;
    this.shooter = shooter;
    // this.climber = climb;
    // this.manager = manager;

    intakeStateSupplier = intake::getIntakeState;

    BooleanSupplier threshold = shooter::isFlywheelAtThreshold;
    // BooleanSupplier state =
    //     () ->
    //         shooter.shooterState.equals(ShooterState.SHOOT_FIXED)
    //             || shooter.shooterState.equals(ShooterState.SHOOT_HUB)
    //             || shooter.shooterState.equals(ShooterState.SHOOT_PASS_L)
    //             || shooter.shooterState.equals(ShooterState.SHOOT_PASS_R);

    BooleanSupplier ramp = () -> threshold.getAsBoolean() && shootSupply.getAsBoolean();

    flywheelRamp = new Trigger(ramp);
    flywheelRamp.whileTrue(
        Commands.run(
            () -> {
              indexer.setIndexerState(IndexerState.INDEXING);
            }));
    // flywheelRamp.onFalse(indexCommand(IndexerState.IDLE));
  }

  public Command intakeCommand(IntakeState state) {
    return Commands.runOnce(
        () -> {
          intake.setIntakeState(state);
        });
  }

  public Command shootCommand() {
    return Commands.parallel(
        shooter.startShooter(),
        Commands.run(
            () -> {
              if (!intakeStateSupplier.get().equals(IntakeState.INTAKING)
                  && !intakeStateSupplier.get().equals(IntakeState.OUTTAKING))
                intake.setIntakeState(IntakeState.AGITATE);
              transfer.setTransferState(TransferState.TRANSFERRING);
            }));
  }

  public Command stopShootCommand() {
    return Commands.parallel(
        shooter.idleShooter(),
        Commands.runOnce(
            () -> {
              if (!intakeStateSupplier.get().equals(IntakeState.INTAKING))
                intake.setIntakeState(IntakeState.IDLE);
              transfer.setTransferState(TransferState.IDLE);
              indexer.setIndexerState(IndexerState.IDLE);
            }));
  }

  public Command stopAgitateCommand() {
    return Commands.runOnce(
        () -> {
          if (!intakeStateSupplier.get().equals(IntakeState.INTAKING))
            intake.setIntakeState(IntakeState.IDLE);
        });
  }

  public Command indexCommand(IndexerState state) {
    return Commands.runOnce(
        () -> {
          indexer.setIndexerState(state);
        });
  }

  // public Command climbCommand(ClimbState state) {
  // Command initial =
  // Commands.runOnce(
  // () -> {
  // climber.setClimbState(state);
  // });
  // if (state != ClimbState.STOW) {
  // return initial.andThen(
  // () -> {
  // shooter.setShooterState(ShooterState.IDLE);
  // intake.setIntakeState(IntakeState.STOW);
  // indexer.setIndexerState(IndexerState.IDLE);
  // transfer.setTransferState(TransferState.IDLE);
  // });
  // }
  // return initial;
  // }

  // public Command runIntake() {
  // return manager.setIntakeCommand(IntakeManagerState.INTAKING);
  // }

  // public Command stopIntake() {
  // return manager.setIntakeCommand(IntakeManagerState.IDLE);
  // }

  // public Command runShoot() {
  // if (manager.robotState != RobotScoringState.CLIMBING)
  // return manager.toStateCommand(correctShooterState());
  // return Commands.none();
  // }

  // public Command stopShoot() {
  // if (manager.robotState != RobotScoringState.CLIMBING) {
  // return manager.toStateCommand(RobotScoringState.IDLE);
  // }

  // return Commands.none();
  // }

  // public Command toggleFixedShooting(boolean newBool) {
  // return Commands.runOnce(
  // () -> {
  // fixedShooting = newBool;
  // });
  // }

  // public RobotScoringState correctShooterState() {
  // return fixedShooting ? RobotScoringState.FIXED_SHOOTING :
  // RobotScoringState.SHOOTING;
  // }
}
