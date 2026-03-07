package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.indexer.IndexerConstants.SpindexerHardware;
import frc.robot.subsystems.indexer.IndexerConstants.SpindexerSimulationConfiguration;

public class IndexerSpindexerIOSim implements IndexerSpindexerIO {
  private final double kLoopPeriodSec;

  private final DCMotorSim spindexerMotor;

  private double appliedVoltage = 0.0;

  private double wheelRadius;

  public IndexerSpindexerIOSim(
      double loopPeriodSec,
      SpindexerHardware hardware,
      SpindexerSimulationConfiguration configuration) {
    spindexerMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                configuration.motorType(), configuration.measurementStdDevs(), hardware.gearing()),
            configuration.motorType());
    kLoopPeriodSec = loopPeriodSec;
    wheelRadius = hardware.wheelRadIn();
  }

  @Override
  public void updateInputs(IndexerSpindexerIOInputs inputs) {
    spindexerMotor.update(kLoopPeriodSec);

    inputs.isMotorConnected = true;

    inputs.angularVelocity = spindexerMotor.getAngularVelocity();
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
    spindexerMotor.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    spindexerMotor.setAngularVelocity(velocity.in(RadiansPerSecond));
  }

  @Override
  public void setTangentialVelocity(LinearVelocity velocity) {
    spindexerMotor.setAngularVelocity(velocity.in(InchesPerSecond) / wheelRadius);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
