package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.visualizers.PivotVisualizer.PivotVisualizerConfiguration;

public class IntakeConstants {

    public record IntakePivotHardware(int motorId, double gearing) {}

    public record IntakeRollerHardware(int motorId, double gearing) {}

    public record IntakePivotGains(
        // Feedback control
        double p,
        double i,
        double d,
        // Motion magic constraints
        double maxVelocityRotationsPerSecond,
        double maxAccelerationRotationsPerSecondSquared,
        double jerkRotationsPerSecondCubed,
        // Pivot feedforward values
        double s,
        double v,
        double a,
        double g) {}

    public record IntakePivotTalonFXConfiguration(
        boolean invert,
        boolean enableStatorCurrentLimit,
        boolean enableSupplyCurrentLimit,
        double statorCurrentLimitAmps,
        double supplyCurrentLimitAmps,
        double peakForwardVoltage,
        double peakReverseVoltage,
        NeutralModeValue neutralMode) {}

    public record IntakeRollerTalonFXConfiguration(
        boolean invert,
        boolean enableStatorCurrentLimit,
        boolean enableSupplyCurrentLimit,
        double statorCurrentLimitAmps,
        double supplyCurrentLimitAmps,
        double peakForwardVoltage,
        double peakReverseVoltage,
        NeutralModeValue neutralMode) {}

    public record IntakePivotSimulationConfiguration(
        DCMotor motorType,
        double momentOfInertiaJKgMetersSquared,
        double ligamentLengthMeters,
        Rotation2d minPosition,
        Rotation2d maxPosition,
        boolean simulateGravity,
        Rotation2d initialPosition,
        double measurementStdDevs) {}

    /**
     * See this comment and documentation about the units if x, y, w, h
     * https://github.com/Prosper-FRC/robot-reefscape-common/pull/20#discussion_r1947983003
     * https://grapplerobotics.au/product/lasercan/
     */
    public record SensorConfiguration(
        int sensorId, double detectionThresholdMilimeters, int x, int y, int w, int h) {}

    public record SimulationConfiguration(DCMotor motorType, double measurementStdDevs) {}

    /* Intake constants */

    /** The frequency that telemetry form the motor is pushed to the CANBus */
    public static final double kStatusSignalUpdateFrequencyHz = 100.0;

    public static final IntakeRollerHardware kRollerMotorHardware =
        new IntakeRollerHardware(
            42, // Motor CAN ID
            18.0 / 24.0 // Gearing
            );

    public static final SimulationConfiguration kIntakeRollerSimulationConfiguration =
        new SimulationConfiguration(DCMotor.getKrakenX60(1), 0.0002);

    /* Pivot constants */

    public static final Rotation2d kMinPivotPosition = Rotation2d.fromRotations(-0.29);
    public static final Rotation2d kMaxPivotPosition = Rotation2d.fromRotations(0.1);

    public static final Rotation2d kPivotPositionTolerance = Rotation2d.fromRotations(0.01);

    public static final double kPivotGearing = 1.0 / 3.0; // TODO Check this value

    public static final double kRollerIntakingVoltage = 3.75;

    public static final double kRollerStowVoltage = 1;

    public static final IntakePivotHardware kPivotMotorHardware =
        new IntakePivotHardware(
            41, // CAN ID
            kPivotGearing); // Gear ratio

    public static final IntakePivotGains kPivotGains =
        switch (Constants.currentMode) {
            case REAL -> new IntakePivotGains(5.0, 0.0, 0.0, 1, 2, 20, 0.0, 0.0, 0.0, -0.4);
            case SIM -> new IntakePivotGains(550.0, 0.0, 0.0, 10.0, 4.0, 0.0, 0.0, 0.17, 0.06, 0.01);
            default -> new IntakePivotGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        };

    public static final IntakePivotTalonFXConfiguration kPivotMotorConfiguration =
        new IntakePivotTalonFXConfiguration(
            false, // Invert
            true, // Enable stator current limiting
            true, // Enable supply current limiting
            60.0, // Stator limit
            50.0, // Supply limit
            12.0, // Peak forward voltage
            -12.0, // Peak reverse voltage
            NeutralModeValue.Brake); // Idle mode

    public static final IntakeRollerTalonFXConfiguration kRollerMotorConfiguration =
        new IntakeRollerTalonFXConfiguration(
            true, // Invert
            true, // Enable stator current limiting
            true, // Enable supply current limiting
            60.0, // Stator limit
            50.0, // Supply limit
            12.0, // Peak forward voltage
            -12.0, // Peak reverse voltage
            NeutralModeValue.Brake); // Idle mode

    // Pivot mass: 2.6553 kg
    // Distance from COM: ~14in
    public static final IntakePivotSimulationConfiguration kPivotSimulationConfiguration =
        new IntakePivotSimulationConfiguration(
            DCMotor.getKrakenX60(1), // Motor type and count
            0.009261, // Moment of inertia (calculated from latest CAD)
            Units.inchesToMeters(20.0), // Ligament length meters
            kMinPivotPosition, // Min position
            kMaxPivotPosition, // Max position
            false, // Simulate gravity
            new Rotation2d(), // Initial position
            0.002); // Std devs

    public static final PivotVisualizerConfiguration kPivotVisualizerConfiguration =
        new PivotVisualizerConfiguration(
            new Pair<Double, Double>(1.0, 3.0),
            "boxPivotRoot",
            new Pair<Double, Double>(0.75, 2.0),
            "boxPivotLigament",
            Rotation2d.fromDegrees(270.0),
            Units.inchesToMeters(20.0),
            new Rotation2d());
}
