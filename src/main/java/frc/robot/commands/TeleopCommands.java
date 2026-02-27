package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.geometry.AllianceFlipUtil;

public class TeleopCommands {
  public enum ShooterState {
    INACTIVE, // This state still keeps the regulator
    SPINUP, // This is the time needed to reach the desired flywheel velocity
    SCORE // This starts the Spindexer and kicker
    // Note that this state encompasses the Spindexer, Transfer and Shooter + constant tracking
  }

  private CommandXboxController controller;
  private Drive drive;
  private Shooter shooter;

  public TeleopCommands(
      Shooter shooter,
      Drive drive,
      CommandXboxController controller) {
    this.controller = controller;
    this.shooter = shooter;
    this.drive = drive;
  
  }

  
}
