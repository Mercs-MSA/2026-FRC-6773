package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;

public class ShooterConstants {

  public static final Rotation2d turretMaxLimit = new Rotation2d(Math.PI);
  public static final Rotation2d turretMinLimit = new Rotation2d(-Math.PI);

  public static final Rotation2d hoodMaxLimit = new Rotation2d(-1);
  public static final Rotation2d hoodMinLimit = new Rotation2d(1);

  public static final Rotation2d turretPositionTolerance = new Rotation2d(Math.toRadians(0.5));

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
      double p,
      double i,
      double d,
      double v,
      double a,
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSquared,
      double jerkMetersPerSecondCubed) {}

  public record HoodGains(double p, double i, double d, double v, double a) {}

  public record FlywheelTalonFXConfiguration(
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
      new ShooterFlywheelHardware(56, 57, 6000 / 4800);

  public static final ShooterHoodHardware hoodHardware =
      new ShooterHoodHardware(
          55, 1 // TODO:  Check and update
          );

  public static final ShooterTurretHardware turretHardware =
      new ShooterTurretHardware(53, 54, 102 / 25);

  public static final TurretGains turretGains =
      switch (Constants.currentMode) {
        case REAL -> new TurretGains(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0);
        case SIM -> new TurretGains(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0);
        default -> new TurretGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0);
      };
}
