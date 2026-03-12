package frc.robot.subsystems.climb;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;

public class ClimbConstants {
  // TODO: CHECK FOLLOWING VOLTAGES AND POSITIONS
  public static final double climbVoltage = 15.0;
  public static final double descendClimbVoltage = -15.0;

  public static final double stowPos = 0.0;
  public static final double L1Pos = 1.0;

  public record ClimbHardware(int ClimbID, double gearing) {}

  public record ClimbTalonFXConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public record KickerGains(double p, double i, double d, double v, double a, double s, double g) {}

  public record ClimbSimulationConfiguration(DCMotor motorType, double measurementStdDevs) {}

  public static final ClimbHardware climbHardware = new ClimbHardware(61, 100);

  public static final ClimbTalonFXConfiguration climbTalonFXConfiguration =
      new ClimbTalonFXConfiguration(
          false, // Invert
          true, // Enable stator current limiting
          true, // Enable supply current limiting
          60.0, // Stator limit
          50.0, // Supply limit
          12.0, // Peak forward voltage
          -12.0, // Peak reverse voltage
          NeutralModeValue.Coast); // Idle mode

  public static final ClimbSimulationConfiguration climbSimulationConfiguration =
      new ClimbSimulationConfiguration(DCMotor.getKrakenX60(1), 0.002);

  /** The frequency that telemetry form the motor is pushed to the CANBus */
  public static final double statusSignalUpdateFrequencyHz = 100.0;
}
