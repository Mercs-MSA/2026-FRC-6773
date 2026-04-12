package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;

public class IndexerConstants {

  public record SpindexerHardware(int motorID, double gearing, double wheelRadIn) {}

  public record KickerHardware(int motorID, double gearing, double wheelRadIn) {}

  public record SpindexerGains(double p, double i, double d, double v, double a) {}

  public record KickerGains(double p, double i, double d, double v, double a) {}

  public record SpindexerTalonFXConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public record KickerTalonFXConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public record SpindexerSimulationConfiguration(DCMotor motorType, double measurementStdDevs) {}

  public record KickerSimulationConfiguration(DCMotor motorType, double measurementStdDevs) {}

  public static final SpindexerHardware spindexerHardware =
      new SpindexerHardware( // TODO: Gearing
          43, 5 * (30 / 18), 3);

  public static final KickerHardware kickerHardware =
      new KickerHardware( // TODO: Gearing
          44, 24d / 18d, 2);

  public static final SpindexerGains spindexerGains = new SpindexerGains(0, 0, 0, 0.105, 0);

  public static final KickerGains kickerGains = new KickerGains(0, 0, 0, 0.105, 0);

  public static final SpindexerTalonFXConfiguration spindexerTalonFXConfiguration =
      new SpindexerTalonFXConfiguration(
          false, // Invert
          true, // Enable stator current limiting
          true, // Enable supply current limiting
          65.0, // Stator limit
          55.0, // Supply limit
          12.0, // Peak forward voltage
          -12.0, // Peak reverse voltage
          NeutralModeValue.Coast); // Idle mode

  public static final KickerTalonFXConfiguration kickerTalonFXConfiguration =
      new KickerTalonFXConfiguration(
          false, // Invert
          true, // Enable stator current limiting
          true, // Enable supply current limiting
          100.0, // Stator limit
          90.0, // Supply limit
          12.0, // Peak forward voltage
          -12.0, // Peak reverse voltage
          NeutralModeValue.Coast); // Idle mode

  public static final SpindexerSimulationConfiguration spindexerSimulationConfiguration =
      new SpindexerSimulationConfiguration(DCMotor.getKrakenX44(1), 0.002);

  public static final KickerSimulationConfiguration kickerSimulationConfiguration =
      new KickerSimulationConfiguration(DCMotor.getKrakenX44(1), 0.002);

  /** The frequency that telemetry form the motor is pushed to the CANBus */
  public static final double statusSignalUpdateFrequencyHz = 100.0;
}
