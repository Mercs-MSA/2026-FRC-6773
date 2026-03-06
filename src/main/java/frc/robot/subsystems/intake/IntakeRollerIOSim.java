package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intake.IntakeConstants.RollerHardware;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSimulationConfiguration;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private final double kLoopPeriodSec;

  private final DCMotorSim rollerMotor;

  private double appliedVoltage = 0.0;

  public IntakeRollerIOSim(
      double loopPeriodSec, RollerHardware hardware, IntakeSimulationConfiguration configuration) {
    rollerMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                configuration.motorType(), configuration.measurementStdDevs(), hardware.gearing()),
            configuration.motorType());
    kLoopPeriodSec = loopPeriodSec;
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    rollerMotor.update(kLoopPeriodSec);

    inputs.isMotorConnected = true;

    inputs.velocityRotPerSec = rollerMotor.getAngularVelocity().in(RotationsPerSecond);
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = 0.0;
    inputs.statorCurrentAmps = 0.0;
    inputs.temperatureCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    rollerMotor.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
