package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class TeleopCommands {
  private CommandXboxController controller;
  private Intake intake;

  public TeleopCommands(Intake intake, CommandXboxController controller) {
    this.intake = intake;
    this.controller = controller;
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
}
