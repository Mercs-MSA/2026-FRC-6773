package frc.robot.subsystems.drive.controllers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.utils.math.AllianceFlipUtil;

/**
 * Utility class for selecting goal poses based on strategy and current robot position.
 * Replace the strategies and poses with game-specific logic for your season.
 */ 
public class GoalPoseChooser {
    public static enum CHOOSER_STRATEGY {
        kTest,
        kScoringPosition,
        kCustom,
        kIntakePosition,
        kAutoPosition
    }

    public static enum SIDE {
        LEFT, RIGHT, CENTER
    }

    private static Pose2d customGoal = FieldConstants.kScoringPosition1;
    private static SIDE side = SIDE.LEFT;

    /**
     * Gets a goal pose based on the strategy and current robot position.
     * Replace with game-specific logic for your season.
     */
    public static Pose2d getGoalPose(CHOOSER_STRATEGY strategy, Pose2d robotPose) {
        switch(strategy) {
            case kTest:
                return new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(60));
            case kScoringPosition:
                return getScoringPose(robotPose);
            case kCustom:
                return customGoal;
            case kIntakePosition:
                return getIntakePose(robotPose);
            case kAutoPosition:
                return getAutonomousStartPose(robotPose);
        }
        return new Pose2d();
    }

    /**
     * Example scoring position selection based on robot location.
     * Replace with game-specific scoring logic.
     */
    public static Pose2d getScoringPose(Pose2d robotPose) {
        // Simple example: choose scoring position based on Y coordinate
        if (robotPose.getY() < Constants.kFieldWidthMeters / 2.0) {
            return AllianceFlipUtil.apply(
                side.equals(SIDE.LEFT) ? FieldConstants.kScoringPosition1 : FieldConstants.kScoringPosition2);
        } else {
            return AllianceFlipUtil.apply(FieldConstants.kScoringPosition2);
        }
    }

    /**
     * Example intake position selection.
     * Replace with game-specific intake logic.
     */
    public static Pose2d getIntakePose(Pose2d robotPose) {
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            return AllianceFlipUtil.apply(
                (robotPose.getY() < Constants.kFieldWidthMeters / 2.0) ? 
                FieldConstants.kIntakePosition1 : FieldConstants.kIntakePosition2);
        } else {
            return ((robotPose.getY() < Constants.kFieldWidthMeters / 2.0) ? 
                FieldConstants.kIntakePosition2 : FieldConstants.kIntakePosition1);
        }
    }

    /**
     * Example autonomous starting position selection.
     * Replace with game-specific auto start logic.
     */
    public static Pose2d getAutonomousStartPose(Pose2d robotPose) {
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            switch(side) {
                case LEFT:
                    return FieldConstants.kBlueStartLeft;
                case RIGHT:
                    return FieldConstants.kBlueStartRight;
                case CENTER:
                default:
                    return FieldConstants.kBlueStartCenter;
            }
        } else {
            switch(side) {
                case LEFT:
                    return FieldConstants.kRedStartLeft;
                case RIGHT:
                    return FieldConstants.kRedStartRight;
                case CENTER:
                default:
                    return FieldConstants.kRedStartCenter;
            }
        }
    }

    public static void updateSideStuff() {
        Logger.recordOutput("Drive/SelectedSide", side);
    }

    /**
     * Sets a custom goal pose.
     */
    public static Command setGoalCommand(Pose2d goalPose) {
        return Commands.runOnce(()-> customGoal = goalPose);
    }

    /**
     * Sets the preferred side for position selection.
     */
    public static Command setSideCommand(SIDE reefSide) {
        return Commands.runOnce(() -> side = reefSide);
    } 

    public static void setSide(SIDE reefSide) {
        side = reefSide;
    }
}