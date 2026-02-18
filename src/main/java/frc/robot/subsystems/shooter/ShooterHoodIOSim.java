package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterHoodHardware;
import frc.robot.subsystems.shooter.ShooterConstants.SimulationConfiguration;

public class ShooterHoodIOSim implements ShooterHoodIO {
  private final double kLoopPeriodSec;

  private final DCMotorSim hoodMotor;

  // private final DCMotorSim flywheelRight;

  private double appliedVoltage = 0.0;

  public ShooterHoodIOSim(
      double loopPeriodSec, ShooterHoodHardware hardware, SimulationConfiguration configuration) {
    hoodMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                configuration.motorType(), configuration.measurementStdDevs(), hardware.gearing()),
            configuration.motorType());
    kLoopPeriodSec = loopPeriodSec;
  }

  @Override
  public void updateInputs(ShooterHoodIOInputs inputs) {
    hoodMotor.update(kLoopPeriodSec);
    // flywheelRight.update(kLoopPeriodSec);

    inputs.isMotorConnected = true;

    inputs.position = Rotation2d.fromRotations(hoodMotor.getAngularPositionRotations());
    // inputs.rightVelocityRotPerSec = flywheelRight.getAngularVelocityRPM() / 60.0;
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = 0.0;
    inputs.statorCurrentAmps = 0.0;
    inputs.temperatureCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    hoodMotor.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setPosition(Rotation2d goalPosition) {
    hoodMotor.setAngle(goalPosition.getRadians());
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
