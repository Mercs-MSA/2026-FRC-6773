package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTurretHardware;
import frc.robot.subsystems.shooter.ShooterConstants.SimulationConfiguration;

public class ShooterTurretIOSim implements ShooterTurretIO {
  private final double kLoopPeriodSec;

  private final DCMotorSim turretMotor;

  // private final DCMotorSim flywheelRight;

  private double appliedVoltage = 0.0;

  public ShooterTurretIOSim(
      double loopPeriodSec, ShooterTurretHardware hardware, SimulationConfiguration configuration) {
    turretMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                configuration.motorType(), configuration.measurementStdDevs(), hardware.gearing()),
            configuration.motorType());
    kLoopPeriodSec = loopPeriodSec;
  }

  @Override
  public void updateInputs(ShooterTurretIOInputs inputs) {
    turretMotor.update(kLoopPeriodSec);
    // flywheelRight.update(kLoopPeriodSec);

    inputs.isMotorConnected = true;

    inputs.position = Rotation2d.fromRotations(turretMotor.getAngularPositionRotations());
    inputs.velocityRotPerSec = turretMotor.getAngularVelocityRPM() / 60.0;
    // inputs.rightVelocityRotPerSec = flywheelRight.getAngularVelocityRPM() / 60.0;
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = 0.0;
    inputs.statorCurrentAmps = 0.0;
    inputs.temperatureCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    turretMotor.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setPosition(Rotation2d goalPosition) {
    turretMotor.setAngle(goalPosition.getRadians());
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void setTurretSetpoint(Angle angle, AngularVelocity velocity) {
    turretMotor.setAngle(angle.in(Radians));
    turretMotor.setAngularVelocity(velocity.in(RadiansPerSecond));
  }
}
