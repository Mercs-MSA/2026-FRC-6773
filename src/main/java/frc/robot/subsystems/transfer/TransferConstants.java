package frc.robot.subsystems.transfer;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;

public class TransferConstants {

  public record TransferHardware(int transferID, double gearing, double wheelRadIn) {}

  public record TransferGains(double p, double i, double d, double v, double a) {}

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

  public static final TransferHardware transferHardware =
      new TransferHardware( // TODO: Gearing
          52, 18d / 24d, 1.5);

  public static final TransferGains transferGains = new TransferGains(0, 0, 0, 0.1, 0);

  public static final TransferTalonFXConfiguration transferTalonFXConfiguration =
      new TransferTalonFXConfiguration(
          false, // Invert
          true, // Enable stator current limiting
          true, // Enable supply current limiting
          60.0, // Stator limit
          50.0, // Supply limit
          12.0, // Peak forward voltage
          -12.0, // Peak reverse voltage
          NeutralModeValue.Coast); // Idle mode

  public static final TransferSimulationConfiguration transferSimulationConfiguration =
      new TransferSimulationConfiguration(DCMotor.getKrakenX44(1), 0.002);

  /** The frequency that telemetry form the motor is pushed to the CANBus */
  public static final double statusSignalUpdateFrequencyHz = 100.0;
}
