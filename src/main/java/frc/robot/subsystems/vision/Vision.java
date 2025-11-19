package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.utils.debugging.LoggedTunableNumber;

import static frc.robot.subsystems.vision.VisionConstants.*;

/**
 * Vision subsystem for handling AprilTag-based localization from multiple cameras.
 * This class processes per-camera observations (pose estimates, uncertainty, timestamps).
 * Update the field layout and camera configuration for your specific game and robot setup.
 */
public class Vision {
    private final CameraIO[] cameras;
    private final CameraIOInputsAutoLogged[] camerasData;

    private static final LoggedTunableNumber kSingleXYStdev = new LoggedTunableNumber(
        "Vision/kSingleXYStdev", kSingleStdDevs.get(0));
    private static final LoggedTunableNumber kMultiXYStdev = new LoggedTunableNumber(
        "Vision/kMultiXYStdev", kMultiStdDevs.get(0));

    // TODO: Update this field layout for your game/season
    private final AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public Vision(CameraIO[] cameras) {
        Logger.recordOutput("Vision/UseSingleTagTransform", KUseSingleTagTransform);
        this.cameras = cameras;
        this.camerasData = new CameraIOInputsAutoLogged[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            camerasData[i] = new CameraIOInputsAutoLogged(); //new logging for each camera
        }
    }

    /** Updates all cameras each cycle. */
    public void periodic(Pose2d lastRobotPose, Pose2d simOdomPose) {
        for (int i = 0; i < cameras.length; i++) {
            cameras[i].updateInputs(camerasData[i], lastRobotPose, simOdomPose);
            Logger.processInputs("Vision/" + camerasData[i].camName, camerasData[i]);
        }
    }

    /**
     * Builds a set of per-camera vision observations.
     * Each observation includes whether data is valid, the estimated pose,
     * standard deviations (how much to trust the data), and a timestamp.
     */
    public VisionObservation[] getVisionObservations() {
        VisionObservation[] observations = new VisionObservation[cameras.length];

        for (int i = 0; i < camerasData.length; i++) {
            CameraIOInputsAutoLogged camData = camerasData[i];

            // Case: camera has a valid, updated target
            if (camData.hasTarget && camData.hasBeenUpdated) {
                observations[i] = processCameraData(camData);
            } 
            // Case: no target or stale data
            else {
                observations[i] = invalidObservation(camData);
            }
        }
        return observations;
    }

    /** Processes a single camera’s data into a valid VisionObservation. */
    private VisionObservation processCameraData(CameraIOInputsAutoLogged camData) {
        double numberOfTargets = camData.numberOfTargets;
        double avgDistMeters = 0.0;

        // Calculate average distance from valid tags
        for (int r = 0; r < camData.latestTagTransforms.length; r++) {
            if (camData.latestTagTransforms[r] != null) {
                if (camData.latestTagAmbiguities[r] < kAmbiguityThreshold) {
                    avgDistMeters += camData.latestTagTransforms[r].getTranslation().getNorm();
                } else {
                    numberOfTargets -= 1; // discard ambiguous tags
                }
            }
        }

        // If no trustworthy tags remain return an invlid observation
        if (numberOfTargets <= 0) {
            return invalidObservation(camData);
        }

        avgDistMeters /= numberOfTargets;
        Logger.recordOutput("Vision/AvgDistMeters", avgDistMeters);
        double xyScalar = Math.pow(avgDistMeters, 2) / numberOfTargets;
        Logger.recordOutput("Vision/xyScalar", xyScalar);

        // Case: single far-away tag → not reliable
        if (numberOfTargets == 1 && avgDistMeters > 5.5) {
            return invalidObservation(camData);
        }

        // Case: single close tag
        if (numberOfTargets == 1) {
            Pose2d singleTagPose = estimateSingleTagPose(camData);
            return new VisionObservation(
                true,
                singleTagPose,
                VecBuilder.fill(
                    kSingleXYStdev.get() * xyScalar,
                    kSingleXYStdev.get() * xyScalar,
                    Double.MAX_VALUE),
                camData.latestTimestamp,
                camData.camName
            );
        }

        // multiple tags
        return new VisionObservation(
            true,
            camData.latestEstimatedRobotPose.toPose2d(),
            VecBuilder.fill(
                kMultiXYStdev.get() * xyScalar,
                kMultiXYStdev.get() * xyScalar,
                Double.MAX_VALUE),
            camData.latestTimestamp,
            camData.camName
        );
    }

    /** Builds an invalid observation */
    private VisionObservation invalidObservation(CameraIOInputsAutoLogged camData) {
        return new VisionObservation(
            false,
            new Pose2d(),
            VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE),
            camData.latestTimestamp,
            camData.camName
        );
    }

    /** Estimates robot pose from a single */
    private Pose2d estimateSingleTagPose(CameraIOInputsAutoLogged camData) {
        if (KUseSingleTagTransform) {
            return fieldLayout.getTagPose(camData.singleTagAprilTagID).get().toPose2d()
                .plus(new Transform2d(
                        camData.cameraToApriltag.getX(),
                        camData.cameraToApriltag.getY(),
                        camData.cameraToApriltag.getRotation().toRotation2d()))
                .plus(toTransform2d(camData.cameraToRobot.inverse()));
        } else {
            return camData.latestEstimatedRobotPose.toPose2d();
        }
    }

    private Transform2d toTransform2d(Transform3d transform) {
        return new Transform2d(
            transform.getX(),
            transform.getY(),
            transform.getRotation().toRotation2d());
    }

    /** Log. */
    public void logVisionObservation(VisionObservation observation, String state) {
        Logger.recordOutput("Vision/Observation/" + observation.camName() + "/State", state);
        Logger.recordOutput("Vision/Observation/" + observation.camName() + "/Timestamp", observation.timeStamp());
        Logger.recordOutput("Vision/Observation/" + observation.camName() + "/Pose", observation.pose());
        Logger.recordOutput("Vision/Observation/" + observation.camName() + "/hasObserved", observation.hasObserved());
        Logger.recordOutput("Vision/Observation/" + observation.camName() + "/StdDevs", observation.stdDevs());
    }

    public record VisionObservation(
        boolean hasObserved,
        Pose2d pose,
        Vector<N3> stdDevs,
        double timeStamp,
        String camName
    ) {}
}