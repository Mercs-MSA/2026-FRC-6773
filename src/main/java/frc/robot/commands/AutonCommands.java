package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.Optional;

public class AutonCommands {

  private boolean stopRollers = false;
  private boolean stopPivot = false;

  private TeleopCommands teleCommands;
  // public TeleopCommands(Elevator elevator, Intake intake, Manipulator manipulator,
  // CommandXboxController controller) {
  //     kElevator = elevator;
  //     kIntake = intake;
  //     kManipulator = manipulator;
  //     kController = controller;
  //     // kClimb = climb;
  // }

  public AutonCommands(TeleopCommands teleopCommands) {
    this.teleCommands = teleopCommands;
  }

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
        autonCommand.addCommands(getPathCommand("H_Start_HIntake"));
        autonCommand.addCommands(teleCommands.runIntakeFloorPickup());
        autonCommand.addCommands(getPathCommand("H_Intake_HSStart"));
        autonCommand.addCommands(teleCommands.startShoot());
        break;
      case "LEFT":
        autonCommand.addCommands(getPathCommand("D_Start_DIntake"));
        autonCommand.addCommands(teleCommands.runIntakeFloorPickup());
        autonCommand.addCommands(getPathCommand("D_Intake_DSStart"));
        autonCommand.addCommands(teleCommands.startShoot());
        break;
      default:
        DriverStation.reportError("Big oops: Invalid Start Pos", false);
        // Do nothing auton
        break;
    }

    return autonCommand;
  }
}
