package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.transfer.Transfer;

public class TeleopCommands {
  public enum ShooterState {
    INACTIVE, // This state still keeps the regulator
    SPINUP, // This is the time needed to reach the desired flywheel velocity
    SCORE // This starts the Spindexer and kicker
    // Note that this state encompasses the Spindexer, Transfer and Shooter + constant tracking
  }

  public static ShooterState globalState = ShooterState.INACTIVE;

  private CommandXboxController controller;
  private Intake intake;
  private Spindexer spindexer;
  private Transfer transfer;
  private Shooter shooter;

  public TeleopCommands(
      Intake intake,
      Spindexer indexer,
      Transfer transfer,
      Shooter shooter,
      CommandXboxController controller) {

    this.intake = intake;
    this.controller = controller;
    this.shooter = shooter;
    this.spindexer = indexer;
    this.transfer = transfer;
  }

  public Command runShootingSystem(double outputVel) {
    return Commands.runOnce(
        () -> {
          shooter.setFlywheelVelocityRPS(outputVel);
          transfer.setRegulatorVelocity(outputVel);
          transfer.setKickerVelocity(outputVel);
          spindexer.setVelocity(outputVel);
        },
        shooter,
        transfer,
        spindexer);
    // return Commands.runOnce(() ->{
    // 	shooter.setFlywheelVelocityRPS(outputVel);
    // 	transfer.setRegulatorVelocity(outputVel * (3.0 / 4.0));
    // 	transfer.setKickerVelocity(outputVel * (2.0 / 4.0));
    // 	spindexer.setVelocity(outputVel * (1.0 / 4.0));
    // });
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

  public Command runIntakeSlowRollers() {
    return Commands.runOnce(
        () -> {
          intake.slowRollers();
          intake.setBrakeMode(true);
        },
        intake);
  }

  public Command stowIntake() {
    return Commands.runOnce(
        () -> {
          intake.setPivotState(IntakeState.kStow);
          intake.stop(true, false);
        });
  }

  public Command startShoot() {
    return runShootingSystem(12);
  }

  public Command stopShoot() {
    return Commands.runOnce(
        () -> {
          transfer.stop();
          spindexer.stopSpindexer();
          shooter.stop(true, false, false);
        },
        shooter,
        transfer,
        spindexer);
  }
}
