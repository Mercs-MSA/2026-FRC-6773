package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;
import java.util.Optional;

public class AutonCommands extends TeleopCommands {

  private Drive drive;

  public AutonCommands(
      Drive drive, Intake intake, Indexer indexer, Transfer transfer, Shooter shooter
      // ,
      // Climb climb
      ) {
    super(
        drive, intake, indexer, transfer, shooter
        // , climb
        );
    this.drive = drive;
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

  public Command getPathCommand(String pathName, int index) {
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName, index);

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
        // autonCommand.addCommands(getPathCommand("C_Start_Climb"));
        break;

      case "RIGHT_FULL":
        // autonCommand.addCommands((ChoreoTraj.H_Intake_4590_H_BUMP$0));

        autonCommand.addCommands(getPathCommand("H_Start_H_BUMP"));
        autonCommand.addCommands(getPathCommand("H_BumpAllianceNeutral"));
        autonCommand.addCommands(intakeCommand(IntakeState.INTAKING));
        autonCommand.addCommands(getPathCommand("H_BUMP_H_Intake_45"));
        autonCommand.addCommands(getPathCommand("H_Intake_45_D_BUMP"));
        autonCommand.addCommands(intakeCommand(IntakeState.STOW));
        autonCommand.addCommands(getPathCommand("D_BumpNeutralAlliance"));
        autonCommand.addCommands(shootCommand());
        break;
      case "RIGHT_45":
        autonCommand.addCommands(getPathCommand("H_Start_H_BUMP"));
        autonCommand.addCommands(getPathCommand("H_BumpAllianceNeutral"));
        autonCommand.addCommands(intakeCommand(IntakeState.INTAKING));
        autonCommand.addCommands(getPathCommand("H_BUMP_H_Intake_45"));
        autonCommand.addCommands(getPathCommand("H_Intake_45_H_BUMP"));
        autonCommand.addCommands(intakeCommand(IntakeState.STOW));
        autonCommand.addCommands(getPathCommand("H_BumpNeutralAlliance"));
        autonCommand.addCommands(shootCommand());
        break;
      case "LEFT_45":
        autonCommand.addCommands(getPathCommand("D_Start_D_BUMP"));
        autonCommand.addCommands(getPathCommand("D_BumpAllianceNeutral"));
        autonCommand.addCommands(intakeCommand(IntakeState.INTAKING));
        autonCommand.addCommands(getPathCommand("D_BUMP_D_Intake_45"));
        autonCommand.addCommands(getPathCommand("D_Intake_45_D_BUMP"));
        autonCommand.addCommands(intakeCommand(IntakeState.STOW));
        autonCommand.addCommands(getPathCommand("D_BumpNeutralAlliance"));
        autonCommand.addCommands(shootCommand());
        break;
      case "LEFT_FULL":
        autonCommand.addCommands(getPathCommand("D_Start_D_BUMP"));
        autonCommand.addCommands(getPathCommand("D_BumpAllianceNeutral"));
        autonCommand.addCommands(intakeCommand(IntakeState.INTAKING));
        autonCommand.addCommands(getPathCommand("D_BUMP_D_Intake_45"));
        autonCommand.addCommands(getPathCommand("D_Intake_45_H_BUMP"));
        autonCommand.addCommands(intakeCommand(IntakeState.STOW));
        autonCommand.addCommands(getPathCommand("H_BumpNeutralAlliance"));
        autonCommand.addCommands(shootCommand());
        break;

      case "RIGHT_TEST":
        autonCommand.addCommands(getAutonCommandSegments("H_Partial_1Pass"));
        break;
      case "RIGHT_FULL_TEST":
        autonCommand.addCommands(getAutonCommandSegments("H_Full_1Pass"));
        break;
      case "LEFT_FULL_TEST":
        autonCommand.addCommands(getAutonCommandSegments("D_Full_1Pass"));
        break;
      case "LEFT_TEST":
        autonCommand.addCommands(getAutonCommandSegments("D_Partial_1Pass"));
        break;
      default:
        DriverStation.reportError("Big oops: Invalid Start Pos", false);
        // Do nothing auton
        break;
    }

    return autonCommand;
  }

  public static Pose2d swapToCorrectPose(Pose2d pose, Alliance curalliance) {
    if (curalliance == DriverStation.getAlliance().get()) {
      return pose;
    } else {
      return FlippingUtil.flipFieldPose(pose);
    }
  }

  // public Command setInitialPose(ChoreoTraj traj)
  // {
  //   return Commands.runOnce(
  //                           () -> {
  //
  // drive.setPose(swapToCorrectPose(ChoreoTraj.D_Start_Intake45.initialPoseBlue(),
  // Alliance.Blue));
  //                           });
  // }

  public Command getAutonCommandSegments(String overallName) {
    SequentialCommandGroup command = new SequentialCommandGroup();
    command.addCommands(getPathCommand(overallName, 0));
    command.addCommands(intakeCommand(IntakeState.INTAKING));

    command.addCommands(getPathCommand(overallName, 1));
    command.addCommands(intakeCommand(IntakeState.IDLE));

    command.addCommands(getPathCommand(overallName, 2));
    command.addCommands(shootCommand());

    command.addCommands(getPathCommand(overallName, 3));
    command.addCommands(intakeCommand(IntakeState.INTAKING));

    command.addCommands(Commands.waitSeconds(5));
    command.addCommands(intakeCommand(IntakeState.STOW));
    command.addCommands(stopShootCommand());

    return command;
  }
}
