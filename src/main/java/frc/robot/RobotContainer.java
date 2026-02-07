// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutonCommands;
import frc.robot.commands.TeleopCommands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakePivotIOTalonFX;
import frc.robot.subsystems.intake.IntakeRollerIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private Intake intake;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private final AutonCommands autonCommands;
  private final TeleopCommands teleopCommands;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        intake =
            new Intake(
                new IntakePivotIOTalonFX(
                    IntakeConstants.kPivotMotorHardware,
                    IntakeConstants.kPivotMotorConfiguration,
                    IntakeConstants.kPivotGains,
                    IntakeConstants.kStatusSignalUpdateFrequencyHz),
                new IntakeRollerIOTalonFX(
                    IntakeConstants.kRollerMotorHardware,
                    IntakeConstants.kRollerMotorConfiguration,
                    IntakeConstants.kStatusSignalUpdateFrequencyHz));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        break;

      default:
        // Replayed robot, disable IO implementations
        break;
    }
    teleopCommands = new TeleopCommands(intake, controller);
    autonCommands = new AutonCommands();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    controller
        .leftTrigger()
        .onTrue(teleopCommands.runIntakeFloorPickup())
        .onFalse(teleopCommands.runIntakeStow());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
