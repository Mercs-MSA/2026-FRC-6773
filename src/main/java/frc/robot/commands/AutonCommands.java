package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ChoreoFiles.*;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;

public class AutonCommands {

  public Drive kRobotDrive;

  private boolean stopRollers = false;
  private boolean stopPivot = false;

  // public TeleopCommands(Elevator elevator, Intake intake, Manipulator manipulator,
  // CommandXboxController controller) {
  //     kElevator = elevator;
  //     kIntake = intake;
  //     kManipulator = manipulator;
  //     kController = controller;
  //     // kClimb = climb;
  // }

  public AutonCommands(Drive robotDrive) {
    this.kRobotDrive = robotDrive;
  }

  public Command getPathCommand(String pathName) {
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
      ChoreoTraj.Bump.asAutoTraj(null).cmd()
      AutoTrajectory
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
        autonCommand.addCommands(getPathCommand("Bump"));
        break;
      case "RIGHT":
        break;
      case "LEFT":
        break;
      default:
        DriverStation.reportError("Big oops: Invalid Start Pos", false);
        // Do nothing auton
        break;
    }

    return autonCommand;
  }
}
