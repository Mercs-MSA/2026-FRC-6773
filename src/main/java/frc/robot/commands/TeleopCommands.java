package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
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

	// private CommandXboxController controller;
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
		// this.controller = controller;
		this.shooter = shooter;
		this.spindexer = indexer;
		this.transfer = transfer;
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
		return Commands.run(
			() -> {
				// intake.setPivotState(IntakeState.kStow);
				intake.slowRollers();
				intake.setBrakeMode(true);
			},
			intake);
	}

	// public Command startShooting() {
	//   return Commands.runOnce(
	//       () -> {
	//         transfer.startTransfer(12);
	//       });
	// }

	// public Command whileShooting() {
	//   return Commands.run(
	//       () -> {
	//         spindexer.setState(transfer.getState());
	//       });
	// }

	public Command stopShooting() {
		return Commands.run(
			() -> {
				transfer.stop();
				spindexer.stopSpindexer();;
			});
	}

	public Command stopShoot() {
		return Commands.runOnce(
			() -> {
				// shooter.stop(true, false, false);
				transfer.setRegulatorVelocity(0);
			});
	}

	public Command startShoot() {
		return Commands.run(
			() -> {
				// shooter.setFlywheelVelocityRPS(60);
				transfer.setRegulatorVelocity(50);
			});
	}

	public Command trackHub() {
		return shooter.runTrackTargetCommand();
	}

	public Command idleShooter() {
		return Commands.runOnce(
			() -> {
				shooter.setHoodPosition(Rotation2d.fromDegrees(1));
				// shooter.setFlywheelVelocityRPS(10);
				shooter.setTurretVoltage(0);
				transfer.setRegulatorVelocity(10);
			});
	}

	public Command startKick() {
		return Commands.run(
			() -> {
				transfer.setKickerVelocity(40);
				// TODO: ADD SHOOTER
			});
	}

	public Command stopKick() {
		return Commands.run(
			() -> {
				transfer.setKickerVelocity(0);
				// TODO: ADD SHOOTER
			});
	}

	public Command spin(double vel) {
		return Commands.run(
			() -> {
				spindexer.setVelocity(vel);
			},
			spindexer);
	}

	public Command spinAlt() {
		return Commands.run(
			() -> {
				spindexer.setVelocity(-30);
			},
			spindexer);
	}

	public Command spinStop() {
		return Commands.run(
			() -> {
				spindexer.setVelocity(0);
			},
			spindexer);
	}

	public Command stop() {
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

	public Command startTransfer(double shooterSpeed) {
		return Commands.runOnce(
			() -> {
				transfer.startTransfer(shooterSpeed);
			},
			transfer);
	}

	public Command stopTransfer() {
		return Commands.runOnce(
			() -> {
				transfer.stop();
				spindexer.stopSpindexer();
			},
			transfer);
	}

	public Command stopKicker() {
		return Commands.runOnce(
			() -> {
				transfer.setKickerVoltage(0);
			},
			transfer);
	}
}
