package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Constants for the vision subsystem.
 * Update camera names, transforms, and thresholds to match your robot configuration.
 */
public class VisionConstants {
  // AprilTag layout - update for your game/season
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // TODO: Update these camera names to match your robot's camera configuration
  // Camera names must match names configured on coprocessor
  public static String camera0Name = "camera-front";
  public static String camera1Name = "camera-back";
  public static String camera2Name = "camera-left";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  // public static Transform3d robotToCamera0 =
  //     new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
  // public static Transform3d robotToCamera1 =
  //     new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // TODO: Tune these filtering thresholds for your robot
  // Basic filtering thresholds
  public static double maxAmbiguity = 0.2; // AprilTag detection ambiguity threshold
  public static double maxZError = 999; // Maximum Z-axis error (disabled by default)

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // TODO: Tune these standard deviations based on your camera setup and testing
  // Standard deviations for pose estimation confidence
  public static final Vector<N3> kSingleStdDevs = (RobotBase.isReal()) ?
        VecBuilder.fill(0.274375, 0.274375, 5.0) : VecBuilder.fill(0.23, 0.23, 5.0);
    public static final Vector<N3> kMultiStdDevs = (RobotBase.isReal()) ?
        VecBuilder.fill(0.23188, 0.23188, 5.0) : VecBuilder.fill(0.23, 0.23, 5.0);

    public static final double kAmbiguityThreshold = (RobotBase.isReal()) ? 0.2 : 1.0;

    public static final boolean KUseSingleTagTransform = false;

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0  // Camera 2
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
