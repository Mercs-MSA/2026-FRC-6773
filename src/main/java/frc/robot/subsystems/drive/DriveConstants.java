package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotBase;

public class DriveConstants {
    ///////////////////// DRIVE BASE \\\\\\\\\\\\\\\\\\\\\\\
    /* PHYSICAL CONSTANTS */
    public static final double kRobotWidthMeters = 0.9144;
    public static final double kTrackWidthXMeters = 0.6;
    public static final double kTrackWidthYMeters = 0.6;
    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
        new Translation2d(kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0),
        new Translation2d(kTrackWidthXMeters / 2.0, -kTrackWidthYMeters / 2.0),
        new Translation2d(-kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0),
        new Translation2d(-kTrackWidthXMeters / 2.0, -kTrackWidthYMeters / 2.0)
      };

    public static final SwerveDriveKinematics kKinematics =
        new SwerveDriveKinematics(kModuleTranslations);

    public static final double kDrivebaseRadiusMeters =
        Math.hypot(kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0);
    
    /* DRIVEBASE CONSTRAINTS */
    public static final double kMaxLinearSpeedMPS = 5.5;
    public static final double kMaxLinearAccelerationMPSS = 12.0;

    public static final double kMaxRotationSpeedRadiansPS = Math.toRadians(360);
    public static final double kMaxRotationAccelRadiansPS = Math.toRadians(360) * 10;

    public static final double kMaxAzimuthAngularRadiansPS = Math.toRadians(1200);

    /* Plugged into setpoint generator */
    public static final PathConstraints kDriveConstraints = new PathConstraints(
        kMaxLinearSpeedMPS, kMaxLinearAccelerationMPSS, 
        kMaxRotationSpeedRadiansPS, kMaxRotationAccelRadiansPS);

    /* MISC */
    public static final double kDriftRate = RobotBase.isReal() ? 3.0 : 5.57;
    public static final double kSniperSpeed = 0.2;

    public static final boolean kDoExtraLogging = false;

    public static final PIDConstants kPPTranslationPID = new PIDConstants(0.9, 0.0, 0.0);
    public static final PIDConstants kPPRotationPID = new PIDConstants(2.5, 0.0, 0.0);
    
    // TODO: Update this gyro ID to match your robot's configuration
    public static final int kGyroID = 10;

    ///////////////////// MODULES \\\\\\\\\\\\\\\\\\\\\\\
    /* GENERAL SWERVE MODULE CONSTANTS */
    public static final boolean kTurnMotorInvert = false;
    public static final double kAzimuthMotorGearing = 12.1;
    public static final double kDriveMotorGearing = 6.48 / 1.0;
    public static final double kWheelRadiusMeters = 5.08 / 100.0;
    public static final double kWheelCircumferenceMeters = 2 * Math.PI * kWheelRadiusMeters;

    public static final double kPeakVoltage = 12.0;

    public static final double kDriveStatorAmpLimit = 80.0;
    public static final double kDriveFOCAmpLimit = 80.0;
    public static final double kDriveSupplyAmpLimit = 60.0;

    public static final double kAzimuthStatorAmpLimit = 40.0;
    public static final double kAzimuthFOCAmpLimit = -30.0;
    public static final NeutralModeValue kNeutralVal = NeutralModeValue.Brake; 

    public static final ModuleControlConfig kModuleControllerConfigs = RobotBase.isReal() ? 
        new ModuleControlConfig(
            new PIDController(2, 0, 0.0), new SimpleMotorFeedforward(0, 0.0, 0),
            new PIDController(100, 0.0, 0.3), new SimpleMotorFeedforward(0.0, 0.0, 0.0)) :
        new ModuleControlConfig(
            new PIDController(0.1, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 2.36, 0.005), 
            new PIDController(4.5, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 0.0));


    /* MODULE SPECIFIC CONSTANTS */
    // TODO: Update these motor and encoder IDs to match your robot's configuration
    // Drive motor IDs (Kraken X60 motors)
    public static final int kFrontLeftDriveID = 11;
    public static final int kFrontRightDriveID = 12;
    public static final int kBackLeftDriveID = 13;
    public static final int kBackRightDriveID = 14;
    
    // Azimuth (steering) motor IDs (Kraken X60 motors)
    public static final int kFrontLeftAzimuthID = 21;
    public static final int kFrontRightAzimuthID = 22;
    public static final int kBackLeftAzimuthID = 23;
    public static final int kBackRightAzimuthID = 24;
    
    // CANcoder IDs for absolute position feedback
    public static final int kFrontLeftEncoderID = 31;
    public static final int kFrontRightEncoderID = 32;
    public static final int kBackLeftEncoderID = 33;
    public static final int kBackRightEncoderID = 34;
    
    // TODO: Update these encoder offsets to match your robot's calibration
    // These offsets calibrate the absolute encoders to the module's zero position
    // If 180 was added, the person who got the offset had the bevel gears on the wrong side when he did it
    public static final ModuleHardwareConfig kFrontLeftHardware =
        new ModuleHardwareConfig(
            kFrontLeftDriveID, 
            kFrontLeftAzimuthID, 
            kFrontLeftEncoderID,
            Rotation2d.fromRotations(-0.219238));

    public static final ModuleHardwareConfig kFrontRightHardware =
        new ModuleHardwareConfig(
            kFrontRightDriveID, 
            kFrontRightAzimuthID, 
            kFrontRightEncoderID, 
            Rotation2d.fromRotations(-0.095703));

    public static final ModuleHardwareConfig kBackLeftHardware =
        new ModuleHardwareConfig(
            kBackLeftDriveID, 
            kBackLeftAzimuthID, 
            kBackLeftEncoderID,
            Rotation2d.fromRotations(-0.192383));

    public static final ModuleHardwareConfig kBackRightHardware =
        new ModuleHardwareConfig(
            kBackRightDriveID, 
            kBackRightAzimuthID, 
            kBackRightEncoderID,
            Rotation2d.fromRotations(-0.138428));

    public static record ModuleHardwareConfig(
        int driveID, int azimuthID, int encoderID, Rotation2d offset) {}

    public static record ModuleControlConfig(
        PIDController driveController,
        SimpleMotorFeedforward driveFF,
        PIDController azimuthController,
        SimpleMotorFeedforward azimuthFF) {}
}