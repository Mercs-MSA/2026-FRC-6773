package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.transfer.Transfer;
import java.util.Optional;

public class AutonCommands extends TeleopCommands {

  public AutonCommands(
      Drive drive,
      Intake intake,
      Spindexer indexer,
      Transfer transfer,
      Shooter shooter,
      CommandXboxController controller) {
    super(intake, indexer, transfer, shooter, controller);
  }



  public void registerNamedCommands() {}

  public Command getPathCommand(String pathName) {
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);

      // Create a path following command using AutoBuilder. This will also trigger event markers.
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  public Optional<PathPlannerPath> getTraj(String pathName) {
    try {
      return Optional.of(PathPlannerPath.fromChoreoTrajectory(pathName));
    } catch (Exception e) {
      e.printStackTrace();
      return Optional.empty();
    }
  }

  public Command getAutonomousSequence(String startChoice) {
    SequentialCommandGroup autonCommand = new SequentialCommandGroup();

    switch (startChoice) {
      case "CENTER":
        autonCommand.addCommands(getPathCommand("C_Start_Climb"));
        break;
      case "RIGHT":
        autonCommand.addCommands(getPathCommand("H_Start_Intake"));
        autonCommand.addCommands(runIntakeFloorPickup());
        autonCommand.addCommands(getPathCommand("H_Intake_SStart"));
        autonCommand.addCommands(startShoot());
        autonCommand.addCommands(Commands.waitSeconds(2.5));
        autonCommand.addCommands(getPathCommand("H_SStart_Climb"));

        break;
      case "LEFT":
        autonCommand.addCommands(getPathCommand("D_Start_Intake"));
        autonCommand.addCommands(runIntakeFloorPickup());
        autonCommand.addCommands(getPathCommand("D_Intake_SStart"));
        autonCommand.addCommands(startShoot());
        autonCommand.addCommands(Commands.waitSeconds(2.5));
        autonCommand.addCommands(getPathCommand("D_SStart_Climb"));

        break;
      default:
        DriverStation.reportError("Big oops: Invalid Start Pos", false);
        // Do nothing auton
        break;
    }

    return autonCommand;
  }
}
