package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.climb.ClimbConstants.ClimbHardware;
import frc.robot.subsystems.climb.ClimbConstants.ClimbSimulationConfiguration;

public class ClimbIOSim implements ClimbIO {
  private final double kLoopPeriodSec;

  private final DCMotorSim climbMotor;

  private double appliedVoltage = 0.0;

  public ClimbIOSim(
      double loopPeriodSec, ClimbHardware hardware, ClimbSimulationConfiguration configuration) {
    climbMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                configuration.motorType(), configuration.measurementStdDevs(), hardware.gearing()),
            configuration.motorType());
    kLoopPeriodSec = loopPeriodSec;
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    climbMotor.update(kLoopPeriodSec);

    inputs.isMotorConnected = true;

    inputs.position = climbMotor.getAngularPositionRotations();
    inputs.velocityRotPerSec = climbMotor.getAngularVelocity().in(RotationsPerSecond);
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = 0.0;
    inputs.statorCurrentAmps = 0.0;
    inputs.temperatureCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    climbMotor.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setPosition(double goalPosition) {
    climbMotor.setAngle(Radians.convertFrom(goalPosition, Degree));
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
