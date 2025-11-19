# FRC Swerve Drive Template

This repository provides a comprehensive swerve drive template for FRC teams, featuring:
- Complete swerve drive implementation with AdvantageKit logging
- Vision subsystem with AprilTag localization
- Auto-alignment capabilities
- Pathfinding and autonomous support
- Extensive testing and characterization tools

## 🚀 Quick Start Guide

Follow these steps to adapt this template for your robot:

### 1. Hardware Configuration

#### Motor IDs (`DriveConstants.java`)
Update the motor and encoder IDs to match your robot's CAN configuration:

```java
// Drive motor IDs (Kraken X60 motors)
public static final int kFrontLeftDriveID = 11;    // Change to your IDs
public static final int kFrontRightDriveID = 12;
public static final int kBackLeftDriveID = 13;
public static final int kBackRightDriveID = 14;

// Azimuth (steering) motor IDs
public static final int kFrontLeftAzimuthID = 21;  // Change to your IDs
public static final int kFrontRightAzimuthID = 22;
public static final int kBackLeftAzimuthID = 23;
public static final int kBackRightAzimuthID = 24;

// CANcoder IDs
public static final int kFrontLeftEncoderID = 31;  // Change to your IDs
public static final int kFrontRightEncoderID = 32;
public static final int kBackLeftEncoderID = 33;
public static final int kBackRightEncoderID = 34;

// Gyro ID
public static final int kGyroID = 10;              // Change to your gyro ID
```

#### Encoder Offsets
**CRITICAL**: You must calibrate your swerve module encoder offsets:

1. Point all wheels straight forward
2. Record the absolute encoder readings
3. Update the offset values:

```java
public static final ModuleHardwareConfig kFrontLeftHardware =
    new ModuleHardwareConfig(
        kFrontLeftDriveID, 
        kFrontLeftAzimuthID, 
        kFrontLeftEncoderID,
        Rotation2d.fromRotations(YOUR_CALIBRATED_OFFSET)); // Update this!
```

#### Physical Constants
Update robot dimensions and constraints in `DriveConstants.java`:

```java
// Measure your robot's dimensions
public static final double kTrackWidthXMeters = 0.6;  // Distance between left/right wheels
public static final double kTrackWidthYMeters = 0.6;  // Distance between front/back wheels

// Tune these based on your robot's performance
public static final double kMaxLinearSpeedMPS = 5.5;
public static final double kMaxLinearAccelerationMPSS = 12.0;
```

### 2. Field Configuration

#### Game-Specific Positions (`FieldConstants.java`)
Replace the generic field positions with your game's specific coordinates:

```java
// Replace these with actual game piece locations
public static final Pose2d kScoringPosition1 = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(0.0));
public static final Pose2d kScoringPosition2 = new Pose2d(2.0, 7.0, Rotation2d.fromDegrees(0.0));

// Replace with actual intake/pickup positions
public static final Pose2d kIntakePosition1 = new Pose2d(14.0, 1.0, Rotation2d.fromDegrees(180.0));
public static final Pose2d kIntakePosition2 = new Pose2d(14.0, 7.0, Rotation2d.fromDegrees(180.0));
```

#### Auto-Alignment Logic (`GoalPoseChooser.java`)
Implement your game-specific pose selection logic:

```java
public static Pose2d getScoringPose(Pose2d robotPose) {
    // Implement your game-specific logic here
    // Example: Choose nearest scoring position, consider game piece type, etc.
}

public static Pose2d getIntakePose(Pose2d robotPose) {
    // Implement your game-specific intake logic here
}
```

### 3. Vision System

#### Camera Configuration (`VisionConstants.java`)
Update camera names to match your setup:

```java
// Update these to match your camera network table names
public static String camera0Name = "camera-front";    // Change to your camera names
public static String camera1Name = "camera-back";
public static String camera2Name = "camera-left";
```

#### AprilTag Field Layout (`Vision.java`)
Update the field layout for your game season:

```java
// Update this for your game/season
private final AprilTagFieldLayout fieldLayout =
    AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo); // Change year/game
```

#### Camera Calibration
Tune the vision standard deviations based on testing:

```java
// Tune these based on your camera setup and testing
public static final Vector<N3> kSingleStdDevs = 
    VecBuilder.fill(0.274375, 0.274375, 5.0); // X, Y, Theta standard deviations
```

### 4. Control Tuning

#### Drive Control (`DriveConstants.java`)
Tune PID controllers and feedforward values:

```java
// Tune these PID values for your robot
public static final ModuleControlConfig kModuleControllerConfigs = RobotBase.isReal() ? 
    new ModuleControlConfig(
        new PIDController(70, 0.0, 0.0),           // Drive PID - tune these!
        new SimpleMotorFeedforward(2.25, 0.0, 1.0), // Drive FF - tune these!
        new PIDController(100, 0.0, 0.3),          // Azimuth PID - tune these!
        new SimpleMotorFeedforward(0.0, 0.0, 0.0)) : // Azimuth FF - tune these!
    // Simulation values...
```

#### PathPlanner PID
Tune autonomous path following:

```java
// Tune for smooth path following
public static final PIDConstants kPPTranslationPID = new PIDConstants(0.9, 0.0, 0.0);
public static final PIDConstants kPPRotationPID = new PIDConstants(2.5, 0.0, 0.0);
```

### 5. Robot-Specific Subsystems

#### Add Your Subsystems
The template includes only drive and vision. Add your robot's subsystems:

1. Create subsystem classes in `src/main/java/frc/robot/subsystems/`
2. Follow the IO layer pattern used in the drive subsystem
3. Add subsystem initialization to `RobotContainer.java`
4. Create commands in `src/main/java/frc/robot/commands/`

#### Example Subsystem Structure
```
src/main/java/frc/robot/subsystems/
├── arm/
│   ├── Arm.java                 // Main subsystem class
│   ├── ArmIO.java               // Hardware interface
│   ├── ArmIOKraken.java         // Real hardware implementation
│   └── ArmIOSim.java            // Simulation implementation
├── intake/
│   └── ... (similar structure)
└── shooter/
    └── ... (similar structure)
```

### 6. Constants Organization

Update `Constants.java` for your robot:

```java
// Add your robot-specific constants
public static final class ArmConstants {
    public static final int kArmMotorID = 20;
    public static final double kArmGearing = 100.0;
    // ... more arm constants
}

public static final class IntakeConstants {
    public static final int kIntakeMotorID = 30;
    public static final double kIntakeSpeed = 0.8;
    // ... more intake constants
}
```

## 🔧 Testing and Calibration

### 1. Module Characterization
Use the built-in characterization commands to tune your drive:

```java
// In RobotContainer, bind these to test buttons
drive.characterizeLinearMotion()  // For drive motor tuning
drive.characterizeAngularMotion() // For rotation/MOI tuning
```

### 2. Vision Calibration
1. Place robot at known field positions
2. Observe vision pose estimates in AdvantageScope
3. Tune standard deviations based on accuracy
4. Test with single vs multiple AprilTags

### 3. Auto-Alignment Testing
1. Set target poses using `drive.setGoalPose()`
2. Switch to `DRIVE_TO_POSE` state
3. Tune holonomic controller gains if needed

## 📊 Logging and Debugging

This template includes comprehensive logging with AdvantageKit:

1. **AdvantageScope**: View real-time data and replay logs
2. **NetworkTables**: Live tuning of PID values
3. **Pose visualization**: Track robot position on field

### Key Logging Outputs
- `Drive/Swerve/MeasuredStates` - Current module states
- `Drive/Odometry/PoseEstimate` - Robot position
- `Vision/Observation/` - Camera pose estimates
- All tuneable numbers for live adjustment

## 🎯 Game-Specific Implementation

### Example: Implementing Scoring
1. Add scoring positions to `FieldConstants.java`
2. Create scoring strategy in `GoalPoseChooser.java`
3. Add `DRIVE_TO_SCORING` state if needed
4. Create scoring commands that use auto-alignment

### Example: Multi-Piece Autonomous
1. Use Choreo/PathPlanner to create paths
2. Load paths in autonomous commands
3. Use auto-alignment between path segments
4. Integrate with other subsystem actions

## 🏗️ Architecture Overview

This template follows best practices:

- **IO Layer Pattern**: Hardware abstraction for testing/simulation
- **Command-Based Programming**: Clean separation of subsystems and commands
- **AdvantageKit Integration**: Comprehensive logging and replay
- **Tuneable Parameters**: Live tuning during testing
- **Modular Design**: Easy to add/remove subsystems

## 📝 Important Notes

1. **Safety First**: Test all changes incrementally
2. **Backup Configs**: Save working configurations before major changes
3. **Team Training**: Ensure multiple team members understand the codebase
4. **Documentation**: Update this README as you customize for your robot

## 🤝 Contributing

When making improvements to this template:
1. Keep changes generic and reusable
2. Add comprehensive documentation
3. Test in both simulation and on real hardware
4. Follow existing code style and patterns

---

## 🏆 Template Features

### Included Subsystems
- ✅ Swerve Drive with full kinematics
- ✅ Vision with AprilTag localization
- ✅ Auto-alignment to arbitrary poses
- ✅ Pathfinding and autonomous support
- ✅ Characterization and testing tools

### Hardware Support
- ✅ Kraken X60 motors (easily adaptable to other motors)
- ✅ CANcoders for absolute position
- ✅ Pigeon 2 IMU
- ✅ Limelight cameras (adaptable to other vision systems)

### Software Features
- ✅ AdvantageKit logging and replay
- ✅ Live parameter tuning
- ✅ Simulation support
- ✅ Comprehensive testing commands
- ✅ Alliance color handling
- ✅ Field-relative coordinates

Happy coding! 🚀