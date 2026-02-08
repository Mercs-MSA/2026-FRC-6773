package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterConstants {

    public record ShooterFlywheelHardware(
      int flyWheelMotorLeftId,
      int flyWheelMotorRightId,
      double gearing) {}

    public record ShooterHoodHardware(
      int hoodMotorId,
      double gearing) {}

    public record ShooterTurretHardware(
      int turretMotorId,
      double gearing) {}

    public record TurretGains(
        double p,
        double i,
        double d,
        double s,
        double v,
        double a,
        double maxVelocityRotationsPerSecond,
        double maxAccelerationRotationsPerSecondSquared,
        double jerkRotationsPerSecondCubed
    ) {}

     public record FlywheelGains(
        double p,
        double i,
        double d,
        double s,
        double v,
        double a,
        double maxVelocityMetersPerSecond,
        double maxAccelerationMetersPerSecondSquared,
        double jerkMetersPerSecondCubed
    ) {}

    public record HoodGains(
        double p,
        double i,
        double d,
        double s,
        double v,
        double a
    ) {}

    public record FlywheelTalonFXConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

    public record TurretMotorConfiguration (
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

    public record HoodMotorConfiguration (
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

    public static final ShooterFlywheelHardware flywheelHardware = new ShooterFlywheelHardware(
        55,
        56,
        6000/4800
        );

    public static final ShooterHoodHardware hoodHardware = new ShooterHoodHardware(
        54,
        1 //TODO:  Check and update
        );
    
    public static final ShooterTurretHardware turretHardware = new ShooterTurretHardware(
        53, 
        102/25
        );

}

