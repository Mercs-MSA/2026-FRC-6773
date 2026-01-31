package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerHardware;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerSimulationConfiguration;

public class IndexerIOSim implements IndexerIO {

    private final double kLoopPeriodSec;

    private final DCMotorSim kMotor;

    private double appliedVoltage = 0.0;

    public IndexerIOSim(
        double loopPeriodSec,
        IndexerHardware hardware,
        IndexerSimulationConfiguration configuration) {

        kLoopPeriodSec = loopPeriodSec;

        kMotor =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    configuration.motorType(), 
                    configuration.measurementStdDevs(), 
                    hardware.gearing()),
                configuration.motorType());
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
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

}
