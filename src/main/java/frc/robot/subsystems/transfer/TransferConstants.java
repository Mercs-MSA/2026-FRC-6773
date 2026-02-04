package frc.robot.subsystems.transfer;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;

public class TransferConstants {
    public record TransferHardware(int motorId, double gearing) {}

    public record transferGains(
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

    public record TransferTalonFXConfiguration(
        boolean invert,
        boolean enableStatorCurrentLimit,
        boolean enableSupplyCurrentLimit,
        double statorCurrentLimitAmps,
        double supplyCurrentLimitAmps,
        double peakForwardVoltage,
        double peakReverseVoltage,
        NeutralModeValue neutralMode) {}

    public record TransferSimulationConfiguration(DCMotor motorType, double measurementStdDevs) {}

    /**
     * See this comment and documentation about the units if x, y, w, h
     * https://github.com/Prosper-FRC/robot-reefscape-common/pull/20#discussion_r1947983003
     * https://grapplerobotics.au/product/lasercan/
     */
    public record SensorConfiguration(
        int sensorId, double detectionThresholdMilimeters, int x, int y, int w, int h) {}

    /* Intake constants */

    /** The frequency that telemetry form the motor is pushed to the CANBus */
    public static final double kStatusSignalUpdateFrequencyHz = 100.0;

    public static final TransferHardware kTransferRegulatorHardware =
        new TransferHardware(
            67, // TODO: CAN ID
            18.0 / 24.0 // TODO: GEARING
            );

    public static final TransferHardware kTransferKickerHardware =
        new TransferHardware(
            41, // TODO: CAN ID
            18.0 / 24.0 // TODO: GEARING
            );

    public static final double kGearing = 1.0 / 3.0; // TODO Check this value

    public record TransferGains(double p, double i, double d, double v, double s) {}

    public static final TransferGains kRegulatorGains =
        new TransferGains(6, 0, 7, 4, 1); // TODO: Values need to be tuned

    public static final TransferGains kSimulationRegulatorGains =
        new TransferGains(0.1, 0, 0, 0.1, 0.1); // TODO: Unsure if the Spindexer values work for Transfer in sim

    public static final TransferTalonFXConfiguration kTransferConfiguration =
        new TransferTalonFXConfiguration(
            false, // Invert
            true, // Enable stator current limiting
            true, // Enable supply current limiting
            60.0, // Stator limit
            50.0, // Supply limit
            12.0, // Peak forward voltage
            -12.0, // Peak reverse voltage
            NeutralModeValue.Brake); // Idle mode

    public static final TransferSimulationConfiguration kTransferSimulationConfiguration =
        new TransferSimulationConfiguration(
            DCMotor.getKrakenX44(2), // Motor type and count
            0.002); // Std devs

    public static final DoubleSupplier kRegulatorVelocity = () -> {return 4.0;}; //TODO: PUT PROPER VALUE HERE
    public static final DoubleSupplier kKickerVoltage = () -> {return 4.0;}; //TODO: PUT PROPER VALUE HERE
    public static final DoubleSupplier kMinRegulatorVelocityScalar = () -> {return 0.4;}; //percent of the shooter velocity that the transfer velocity has to be so ball reaches shooter at desired speed TODO: PUT PROPER VALUE

}
