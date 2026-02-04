package frc.robot.subsystems.transfer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.transfer.TransferConstants.TransferGains;
import frc.robot.subsystems.transfer.TransferConstants.TransferHardware;
import frc.robot.subsystems.transfer.TransferConstants.TransferSimulationConfiguration;

public class TransferIOSim implements TransferIO { //TODO: Add kicker motor
  private final double kLoopPeriodSec;

  private final DCMotorSim kMotor;

  private double appliedVoltage = 0.0;

  public TransferIOSim(
      double loopPeriodSec,
      TransferHardware hardware,
      TransferGains gains,
      TransferSimulationConfiguration configuration) {

    kLoopPeriodSec = loopPeriodSec;

    kMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                configuration.motorType(), configuration.measurementStdDevs(), hardware.gearing()),
            configuration.motorType());
  }

  @Override
  public void updateInputs(TransferIOInputs inputs) {
    kMotor.update(kLoopPeriodSec);

    inputs.isMotorConnected = true;

    inputs.velocityRotPerSec = kMotor.getAngularVelocityRadPerSec() / (Math.PI * 2);
    inputs.appliedVoltage = appliedVoltage;
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
