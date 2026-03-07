package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.indexer.IndexerConstants.KickerHardware;
import frc.robot.subsystems.indexer.IndexerConstants.KickerSimulationConfiguration;

public class IndexerKickerIOSim implements IndexerKickerIO {
  private final double kLoopPeriodSec;

  private final DCMotorSim kickerMotor;

  private double appliedVoltage = 0.0;

  private double wheelRadius;

  public IndexerKickerIOSim(
      double loopPeriodSec, KickerHardware hardware, KickerSimulationConfiguration configuration) {
    kickerMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                configuration.motorType(), configuration.measurementStdDevs(), hardware.gearing()),
            configuration.motorType());
    kLoopPeriodSec = loopPeriodSec;
    wheelRadius = hardware.wheelRadIn();
  }

  @Override
  public void updateInputs(IndexerKickerIOInputs inputs) {
    kickerMotor.update(kLoopPeriodSec);

    inputs.isMotorConnected = true;

    inputs.angularVelocity = kickerMotor.getAngularVelocity();
    inputs.linearVelocity =
        InchesPerSecond.of(
            inputs.angularVelocity.in(RotationsPerSecond) * wheelRadius * 2 * Math.PI);
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = 0.0;
    inputs.statorCurrentAmps = 0.0;
    inputs.temperatureCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    kickerMotor.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    kickerMotor.setAngularVelocity(velocity.in(RadiansPerSecond));
  }

  @Override
  public void setTangentialVelocity(LinearVelocity velocity) {
    kickerMotor.setAngularVelocity(velocity.in(InchesPerSecond) / wheelRadius);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
