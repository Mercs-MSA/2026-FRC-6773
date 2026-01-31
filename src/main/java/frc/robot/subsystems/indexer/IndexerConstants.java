package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IndexerConstants {
    public record IndexerHardware(int motorId, double gearing) {}

    // public record IndexerGains(
    //     // Feedback control
    //     double p,
    //     double i,
    //     double d,
    //     // Motion magic constraints
    //     double maxVelocityRotationsPerSecond,
    //     double maxAccelerationRotationsPerSecondSquared,
    //     double jerkRotationsPerSecondCubed,
    //     // Pivot feedforward values
    //     double s,
    //     double v,
    //     double a,
    //     double g) {}

    public record IndexerTalonFXConfiguration(
        boolean invert,
        boolean enableStatorCurrentLimit,
        boolean enableSupplyCurrentLimit,
        double statorCurrentLimitAmps,
        double supplyCurrentLimitAmps,
        double peakForwardVoltage,
        double peakReverseVoltage,
        NeutralModeValue neutralMode) {}

    public record IndexerSimulationConfiguration(
        DCMotor motorType,
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

    public static final IndexerHardware kIndexerHardware =
        new IndexerHardware(
            42, // Motor CAN ID
            18.0 / 24.0 // Gearing
            );

    public static final SimulationConfiguration kIndexerSimulationConfiguration =
        new SimulationConfiguration(DCMotor.getKrakenX60(1), 0.0002);

    /* Pivot constants */

    public static final Rotation2d kMinPivotPosition = Rotation2d.fromRotations(-0.29);
    public static final Rotation2d kMaxPivotPosition = Rotation2d.fromRotations(0.1);

    public static final Rotation2d kPivotPositionTolerance = Rotation2d.fromRotations(0.01);

    public static final double kPivotGearing = 1.0 / 3.0; // TODO Check this value

    public static final double kRollerIntakingVoltage = 3.75;

    public static final double kRollerStowVoltage = 1;

    public static final IndexerTalonFXConfiguration kMotorConfiguration =
        new IndexerTalonFXConfiguration(
            false, // Invert
            true, // Enable stator current limiting
            true, // Enable supply current limiting
            60.0, // Stator limit
            50.0, // Supply limit
            12.0, // Peak forward voltage
            -12.0, // Peak reverse voltage
            NeutralModeValue.Brake); // Idle mode

    public static final IndexerSimulationConfiguration kPivotSimulationConfiguration =
        new IndexerSimulationConfiguration(
            DCMotor.getKrakenX60(1), // Motor type and count
            new Rotation2d(), // Initial position
            0.002); // Std devs

    // public static final PivotVisualizerConfiguration kPivotVisualizerConfiguration =
    //     new PivotVisualizerConfiguration(
    //         new Pair<Double, Double>(1.0, 3.0),
    //         "boxPivotRoot",
    //         new Pair<Double, Double>(0.75, 2.0),
    //         "boxPivotLigament",
    //         Rotation2d.fromDegrees(270.0),
    //         Units.inchesToMeters(20.0),
    //         new Rotation2d());
}
