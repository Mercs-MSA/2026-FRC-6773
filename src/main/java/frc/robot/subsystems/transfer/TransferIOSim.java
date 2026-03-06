package frc.robot.subsystems.transfer;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.transfer.TransferConstants.TransferHardware;
import frc.robot.subsystems.transfer.TransferConstants.TransferSimulationConfiguration;

public class TransferIOSim implements TransferIO {
    private final double kLoopPeriodSec;

    private final DCMotorSim transferMotor;

    private double appliedVoltage = 0.0;

    private double wheelRadius;

    public TransferIOSim(
            double loopPeriodSec, TransferHardware hardware, TransferSimulationConfiguration configuration) {
        transferMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        configuration.motorType(), configuration.measurementStdDevs(), hardware.gearing()),
                configuration.motorType());
        kLoopPeriodSec = loopPeriodSec;
        wheelRadius = hardware.wheelRadIn();
    }

    @Override
    public void updateInputs(TransferIOInputs inputs) {
        transferMotor.update(kLoopPeriodSec);

        inputs.isMotorConnected = true;

        inputs.angularVelocity = transferMotor.getAngularVelocity();
        inputs.linearVelocity = InchesPerSecond
                .of(inputs.angularVelocity.in(RotationsPerSecond) * wheelRadius * 2 * Math.PI);
        inputs.appliedVoltage = appliedVoltage;
        inputs.supplyCurrentAmps = 0.0;
        inputs.statorCurrentAmps = 0.0;
        inputs.temperatureCelsius = 0.0;
    }

    @Override
    public void setVoltage(double volts) {
        appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
        transferMotor.setInputVoltage(appliedVoltage);
    }

    @Override
    public void setAngularVelocity(AngularVelocity velocity) {
        transferMotor.setAngularVelocity(velocity.in(RadiansPerSecond));
    }

    @Override
    public void setTangentialVelocity(LinearVelocity velocity) {
        transferMotor.setAngularVelocity(velocity.in(InchesPerSecond) / wheelRadius);
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }
}
