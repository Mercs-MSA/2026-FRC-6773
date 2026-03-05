package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import java.util.Optional;

public class AutonCommands extends TeleopCommands {

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

  public AutonCommands(Shooter shooter, Drive drive, CommandXboxController controller) {
    super(shooter, drive, controller);
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
      case "LEFT":
        autonCommand.addCommands(getDynamic("D_Start", "Intake_45", "Shoot"));
        break;
      case "RIGHT":
        autonCommand.addCommands(getDynamic("H_Start", "Intake_45", "Shoot"));
        break;
      case "LEFT_FULL":
        autonCommand.addCommands(getDynamic("D_Start", "Intake_45", "H_Shoot"));
        break;
      case "RIGHT_FULL":
        autonCommand.addCommands(getDynamic("H_Start", "Intake_45", "D_Shoot"));
        break;
      case "RIGHT_PLAYER":
        autonCommand.addCommands(getDynamic("H_Start", "H_HumanPlayer", "Intake_45", "Shoot"));
        break;
      case "LEFT_DEPOT":
        autonCommand.addCommands(getDynamic("D_Start", "D_Depot", "Intake_45", "Shoot"));
        break;
    }

    return autonCommand;
  }

  /*
   * Command to automatically make a sequential command based on a list of waypoints
   *
   * If ()
   */
  public Command getDynamic(String... wayPoints) {
    if (wayPoints.length < 1) {
      DriverStation.reportError("1 or less waypoints given", null);
      return null;
    }
    SequentialCommandGroup autonCommand = new SequentialCommandGroup();

    DynamicWaypoint last = new DynamicWaypoint(wayPoints[0]);

    for (int i = 1; i < wayPoints.length; i++) {
      DynamicWaypoint curr;
      if (wayPoints[i].split("_")[0].length() != 1) {
        curr = new DynamicWaypoint(wayPoints[i], last.side);
      } else {
        curr = new DynamicWaypoint(wayPoints[i]);
      }
      if (curr.point.equals(last.point) && curr.side == last.side) {
        continue;
      }

      autonCommand.addCommands(curr.fromLast(last));
      last = curr;
    }

    return autonCommand;
  }

  public class DynamicWaypoint {
    public String point;
    public char side;
    public boolean isAlliance;

    public DynamicWaypoint(String point) {
      if (point.charAt(1) != '_' && !(point.equals("Depot") || point.equals("HumanPlayer"))) {
        DriverStation.reportError("please specify side in point string if there is no side", null);
      } else if (point.equals("HumanPlayer")) {
        this.point = point;
        side = 'H';
      } else if (point.equals("Depot")) {
        this.point = point;
        side = 'D';
      } else {
        if (point.charAt(1) == '_') {
          this.point = point.split("_")[1];
        } else {
          this.point = point;
        }
        this.side = point.charAt(0);

        switch (this.point) {
          case "Start":
          case "Depot":
          case "HumanPlayer":
          case "Shoot":
            this.isAlliance = true;
            break;
          case "Intake_45":
            this.isAlliance = false;
            break;
          default:
            DriverStation.reportError("Unknown waypoint given", null);
        }
      }
    }

    public DynamicWaypoint(String point, char side) {
      if (point.charAt(1) == '_') {
        this.point = point.split("_")[1];
      } else {
        this.point = point;
      }
      if (point.charAt(1) == '_' && point.charAt(0) != side) {
        this.side = point.charAt(0);
      } else {
        this.side = side;
      }

      switch (this.point) {
        case "Start":
        case "Depot":
        case "HumanPlayer":
        case "Shoot":
          this.isAlliance = true;
          break;
        case "Intake_45":
          this.isAlliance = false;
          break;
        default:
          DriverStation.reportError("Unknown waypoint given", null);
      }
    }

    // TODO: ADD LOGIC FOR COMMANDS WHEN THOSE ARE IMPLEMENTED IN TELEOPCOMMANDS
    public Command fromLast(DynamicWaypoint last) {
      SequentialCommandGroup command = new SequentialCommandGroup();

      if (last.isAlliance != isAlliance) {
        command.addCommands(
            getPathCommand(last.side + "_" + last.point + "_" + this.side + "_BUMP"));
        if (last.isAlliance) {
          command.addCommands(getPathCommand(this.side + "_BumpAllianceNeutral"));
        } else {
          command.addCommands(getPathCommand(this.side + "_BumpNeutralAlliance"));
        }

        if (!this.point.equals("Shoot")) {
          command.addCommands(getPathCommand(this.side + "_BUMP_" + this.side + "_" + this.point));
        }
      } else {
        command.addCommands(
            getPathCommand(last.side + "_" + last.point + "_" + this.side + "_" + this.point));
      }

      return command;
    }
  }
}
