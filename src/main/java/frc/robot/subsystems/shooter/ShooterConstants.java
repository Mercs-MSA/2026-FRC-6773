package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.Constants;

public class ShooterConstants {
  public static final Rotation2d turretMaxLimit = Rotation2d.fromDegrees(90);
  public static final Rotation2d turretMinLimit = Rotation2d.fromDegrees(-90);

  public static final Rotation2d hoodMaxLimit = new Rotation2d(-1);
  public static final Rotation2d hoodMinLimit = new Rotation2d(1);

  public static final Rotation2d turretPositionTolerance = new Rotation2d(Math.toRadians(0.5));

  public static Transform3d robotToTurret =
      new Transform3d(-0.19685, 0.13567, 0.44, Rotation3d.kZero);

  public record ShooterFlywheelHardware(
      int flyWheelMotorLeftId, int flyWheelMotorRightId, double gearing) {}

  public record ShooterHoodHardware(int hoodMotorId, double gearing) {}

  public record ShooterTurretHardware(int turretMotorId, int cancoderID, double gearing) {}

  public record TurretGains(
      double p,
      double i,
      double d,
      double s,
      double v,
      double a,
      double maxVelocityRotationsPerSecond,
      double maxAccelerationRotationsPerSecondSquared,
      double jerkRotationsPerSecondCubed) {}

  public record FlywheelGains(
      double p, double i, double d, double s, double v, double a, double g) {}

  public record HoodGains(double p, double i, double d, double s, double v, double a, double g) {}

  public record FlywheelMotorConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public record TurretMotorConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public record HoodMotorConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public record SimulationConfiguration(DCMotor motorType, double measurementStdDevs) {}

  public static final ShooterFlywheelHardware flywheelHardware =
      new ShooterFlywheelHardware(56, 57, 24 / 20);

  public static final ShooterHoodHardware hoodHardware =
      new ShooterHoodHardware(
          55, 163.0 // TODO:  Check and update
          );

  public static final ShooterTurretHardware turretHardware =
      new ShooterTurretHardware(53, 54, 102 / 25);

  public static final TurretMotorConfiguration turretConfigs =
      new TurretMotorConfiguration(false, true, true, 60, 50, 12, -12, NeutralModeValue.Brake);
  public static final FlywheelMotorConfiguration flywheelConfigs =
      new FlywheelMotorConfiguration(true, false, false, 60, 50, 12, -12, NeutralModeValue.Brake);
  public static final HoodMotorConfiguration hoodConfigs =
      new HoodMotorConfiguration(true, false, false, 60, 50, 12, -12, NeutralModeValue.Brake);

  public static final SimulationConfiguration shooterSimConfig =
      new SimulationConfiguration(DCMotor.getKrakenX44(1), 0.002);

  public static final FlywheelGains flywheelGains =
      new FlywheelGains(0.2, 0.0, 0.0, 0.1, 0.126, 0.0, 0.0);
  public static final HoodGains hoodGains = new HoodGains(70, 0, 0, 0.1, 0, 0, 0.3);
  public static final TurretGains turretGains =
      switch (Constants.currentMode) {
        case REAL -> new TurretGains(50, 0.0, 0.5, 3.0, 0.0, 0.0, 0.2, 0.0, 0);
        case SIM -> new TurretGains(5, 0.0, 0.01, 0.0, 0.0, 0.0, 0.2, 0.0, 0);
        default -> new TurretGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0);
      };

  /** The frequency that telemetry form the motor is pushed to the CANBus */
  public static final double kStatusSignalUpdateFrequencyHz = 100.0;
}
