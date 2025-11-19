package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Contains field-relative coordinate constants and utility methods.
 * Replace with specific field coordinates for your game/season.
 */
public class FieldConstants {
    // Example field center point - replace with game-specific coordinates
    public static final Pose2d kFieldCenter = new Pose2d(Constants.kFieldLengthMeters / 2.0, Constants.kFieldWidthMeters / 2.0, Rotation2d.fromDegrees(0.0));
    
    // Example scoring positions - replace with game-specific locations
    public static final Pose2d kScoringPosition1 = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kScoringPosition2 = new Pose2d(2.0, 7.0, Rotation2d.fromDegrees(0.0));
    
    // Example intake/pickup positions - replace with game-specific locations
    public static final Pose2d kIntakePosition1 = new Pose2d(14.0, 1.0, Rotation2d.fromDegrees(180.0));
    public static final Pose2d kIntakePosition2 = new Pose2d(14.0, 7.0, Rotation2d.fromDegrees(180.0));
    
    // Example autonomous starting positions
    public static final Pose2d kBlueStartLeft = new Pose2d(1.5, 1.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kBlueStartCenter = new Pose2d(1.5, 4.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kBlueStartRight = new Pose2d(1.5, 7.0, Rotation2d.fromDegrees(0.0));
    
    public static final Pose2d kRedStartLeft = new Pose2d(14.7, 1.0, Rotation2d.fromDegrees(180.0));
    public static final Pose2d kRedStartCenter = new Pose2d(14.7, 4.0, Rotation2d.fromDegrees(180.0));
    public static final Pose2d kRedStartRight = new Pose2d(14.7, 7.0, Rotation2d.fromDegrees(180.0));
    
    /**
     * Utility method to calculate average of two values.
     */
    public static double average(double a, double b) {
        return (a + b) / 2.0;
    }
}