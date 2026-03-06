package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterFlywheelHardware;
import frc.robot.subsystems.shooter.ShooterConstants.SimulationConfiguration;

public class ShooterFlywheelIOSim implements ShooterFlywheelIO {
  private final double kLoopPeriodSec;

  private final FlywheelSim flywheelLeft;

  private double appliedVoltage = 0.0;

  public ShooterFlywheelIOSim(
      double loopPeriodSec,
      ShooterFlywheelHardware hardware,
      SimulationConfiguration configuration) {
    flywheelLeft =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                configuration.motorType(), configuration.measurementStdDevs(), hardware.gearing()),
            configuration.motorType());
    kLoopPeriodSec = loopPeriodSec;
  }

  @Override
  public void updateInputs(ShooterFlywheelIOInputs inputs) {
    flywheelLeft.update(kLoopPeriodSec);

    inputs.isMotorConnected = true;

    inputs.leftVelocity = flywheelLeft.getAngularVelocity();
    inputs.rightVelocity = inputs.leftVelocity;
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = 0.0;
    inputs.statorCurrentAmps = 0.0;
    inputs.temperatureCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    flywheelLeft.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setVelocityRPS(double velocity) {
    flywheelLeft.setAngularVelocity(velocity);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
