package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSimulationConfiguration;
import frc.robot.subsystems.intake.IntakeConstants.PivotHardware;

public class IntakePivotIOSim implements IntakePivotIO {
  private final double kLoopPeriodSec;

  private final DCMotorSim pivotMotor;

  private double appliedVoltage = 0.0;

  public IntakePivotIOSim(
      double loopPeriodSec, PivotHardware hardware, IntakeSimulationConfiguration configuration) {
    pivotMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                configuration.motorType(), configuration.measurementStdDevs(), hardware.gearing()),
            configuration.motorType());
    kLoopPeriodSec = loopPeriodSec;
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    pivotMotor.update(kLoopPeriodSec);

    inputs.isMotorConnected = true;

    inputs.position = Rotation2d.fromRotations(pivotMotor.getAngularPositionRotations());
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = 0.0;
    inputs.statorCurrentAmps = 0.0;
    inputs.temperatureCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    pivotMotor.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setPosition(Rotation2d goalPosition) {
    pivotMotor.setAngle(goalPosition.getRadians());
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public Angle getPosition() {
    return pivotMotor.getAngularPosition();
  }
}
