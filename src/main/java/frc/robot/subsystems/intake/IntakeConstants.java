package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeConstants {
  public static final Rotation2d pivotMaxLimit = Rotation2d.fromRotations(0.24);
  public static final Rotation2d pivotMinLimit = Rotation2d.fromRotations(-0.05);

  public record RollerHardware(int leftRollerID, int rightRollerID, double gearing) {}

  public record PivotHardware(int pivotID, double gearing) {}

  public record PivotGains(
      double p,
      double i,
      double d,
      double s,
      double v,
      double a,
      double g,
      double maxVelocityRotationsPerSecond,
      double maxAccelerationRotationsPerSecondSquared,
      double jerkRotationsPerSecondCubed) {}

  public record RollerTalonFXConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public record PivotTalonFXConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public record IntakeSimulationConfiguration(DCMotor motorType, double measurementStdDevs) {}

  public static RollerHardware rollerHardware = new RollerHardware(40, 41, 3 * (24d / 18d));

  public static PivotHardware pivotHardware = new PivotHardware(42, 24d / 1d);

  public static PivotGains pivotGains =
      new PivotGains(
          25, // p
          0, // i
          0, // d
          0, // s
          0, // v
          0, // a
          0, // g
          0, // MM Max Velocity
          0, // MM Max Accel
          0 // MM Max Jerk
          );

  public static final PivotTalonFXConfiguration kPivotMotorConfiguration =
      new PivotTalonFXConfiguration(
          true, // Invert
          true, // Enable stator current limiting
          true, // Enable supply current limiting
          60.0, // Stator limit
          50.0, // Supply limit
          12.0, // Peak forward voltage
          -12.0, // Peak reverse voltage
          NeutralModeValue.Coast); // Idle mode

  public static final RollerTalonFXConfiguration kRollerMotorConfiguration =
      new RollerTalonFXConfiguration(
          true, // Invert
          true, // Enable stator current limiting
          true, // Enable supply current limiting
          80.0, // Stator limit
          40.0, // Supply limit
          12.0, // Peak forward voltage
          -12.0, // Peak reverse voltage
          NeutralModeValue.Coast); // Idle mode

  public static final IntakeSimulationConfiguration pivotSimulationConfiguration =
      new IntakeSimulationConfiguration(DCMotor.getKrakenX60(1), 0.002);
  public static final IntakeSimulationConfiguration rollerSimulationConfiguration =
      new IntakeSimulationConfiguration(DCMotor.getKrakenX60(1), 0.002);

  /** The frequency that telemetry form the motor is pushed to the CANBus */
  public static final double kStatusSignalUpdateFrequencyHz = 100.0;
}
