package frc.robot.subsystems.Spindexer;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;

public class SpindexerConstants {
  public record SpindexerHardware(int motorId, double gearing) {}

  // public record SpindexerGains(
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

  public record SpindexerTalonFXConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public record SpindexerSimulationConfiguration(DCMotor motorType, double measurementStdDevs) {}

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

  public static final SpindexerHardware kSpindexerHardware =
      new SpindexerHardware(
          42, // TODO: CAN ID
          18.0 / 24.0 // TODO: GEARING
          );

  public static final double kGearing = 1.0 / 3.0; // TODO Check this value

  public record SpindexerGains(double p, double i, double d, double v, double s) {}

  public static final SpindexerGains kSpindexerGains = new SpindexerGains(1, 1, 1, 1, 1); //TODO: FIX THIS IT'S BAD

  public static final SpindexerTalonFXConfiguration kSpindexerConfiguration =
      new SpindexerTalonFXConfiguration(
          false, // Invert
          true, // Enable stator current limiting
          true, // Enable supply current limiting
          60.0, // Stator limit
          50.0, // Supply limit
          12.0, // Peak forward voltage
          -12.0, // Peak reverse voltage
          NeutralModeValue.Brake); // Idle mode

  public static final SpindexerSimulationConfiguration kSpindexerSimulationConfiguration =
      new SpindexerSimulationConfiguration(
          DCMotor.getKrakenX60(1), // Motor type and count
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
