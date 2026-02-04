package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.spindexer.SpindexerConstants.SpindexerGains;
import frc.robot.subsystems.spindexer.SpindexerConstants.SpindexerHardware;
import frc.robot.subsystems.spindexer.SpindexerConstants.SpindexerSimulationConfiguration;

public class SpindexerIOSim implements SpindexerIO {
  private final double kLoopPeriodSec;

  private final DCMotorSim kMotor;

  private double appliedVoltage = 0.0;

  public SpindexerIOSim(
      double loopPeriodSec,
      SpindexerHardware hardware,
      SpindexerGains gains,
      SpindexerSimulationConfiguration configuration) {

    kLoopPeriodSec = loopPeriodSec;

    kMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                configuration.motorType(), configuration.measurementStdDevs(), hardware.gearing()),
            configuration.motorType());
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    kMotor.update(kLoopPeriodSec);

    inputs.isMotorConnected = true;

    inputs.velocityRotPerSec = kMotor.getAngularVelocityRadPerSec() / (Math.PI * 2);
    inputs.appliedVoltage = appliedVoltage;
    // These are 0 cause we don't care about these in simulation
    inputs.supplyCurrentAmps = 0.0;
    inputs.statorCurrentAmps = 0.0;
    inputs.temperatureCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);

    kMotor.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void setVelocity(double velocity) {
    kMotor.setAngularVelocity(velocity * 2 * Math.PI);
  }

  @Override
  public double getVelocity() {
    return kMotor.getAngularVelocityRPM() / 60;
  }
}
