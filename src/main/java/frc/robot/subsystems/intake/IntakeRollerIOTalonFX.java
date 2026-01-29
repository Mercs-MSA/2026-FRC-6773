package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRollerHardware;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRollerTalonFXConfiguration;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
    private final TalonFX kMotor;

    private TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();



    //logged data for roller:
    private StatusSignal<AngularVelocity> velocityRotPerSec;
    private StatusSignal<Current> supplyAmps;
    private StatusSignal<Current> statorAmps;
    private StatusSignal<Voltage> appliedVolts;
    private StatusSignal<Temperature> temperatureCelsius;

    private final VoltageOut kVoltageControl = new VoltageOut(0.0);

    public IntakeRollerIOTalonFX(
        String canbus,
        IntakeRollerHardware hardware,
        IntakeRollerTalonFXConfiguration configuration,
        double statusSignalUpdateFrequency
    ) {
        kMotor = new TalonFX(hardware.motorId(), canbus);


        motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = configuration.enableSupplyCurrentLimit();
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = configuration.supplyCurrentLimitAmps();
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = configuration.enableStatorCurrentLimit();
        motorConfiguration.CurrentLimits.StatorCurrentLimit = configuration.statorCurrentLimitAmps();
        motorConfiguration.Voltage.PeakForwardVoltage = configuration.peakForwardVoltage();
        motorConfiguration.Voltage.PeakReverseVoltage = configuration.peakReverseVoltage();
        motorConfiguration.MotorOutput.Inverted = configuration.invert() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = configuration.neutralMode();

        velocityRotPerSec = kMotor.getVelocity();
        appliedVolts = kMotor.getMotorVoltage();
        supplyAmps = kMotor.getSupplyCurrent();
        statorAmps = kMotor.getStatorCurrent();
        temperatureCelsius = kMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(statusSignalUpdateFrequency,
            velocityRotPerSec,
            appliedVolts,
            supplyAmps,
            supplyAmps,
            statorAmps,
            temperatureCelsius);

        kMotor.optimizeBusUtilization(0.0, 1.0); //TODO: What is this?

    } 

    public IntakeRollerIOTalonFX(
        IntakeRollerHardware hardware,
        IntakeRollerTalonFXConfiguration configuration,
        double statusSignalUpdateFrequency) {

        // Assumes the rio is the CANBus
        this("rio", hardware, configuration, statusSignalUpdateFrequency);
    }
    
    public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.isMotorConnected = BaseStatusSignal.refreshAll(
      velocityRotPerSec,
      appliedVolts,
      supplyAmps,
      statorAmps,
      temperatureCelsius).isOK();

    inputs.velocityRotPerSec = velocityRotPerSec.getValueAsDouble();
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyAmps.getValueAsDouble();
    inputs.statorCurrentAmps = statorAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
  }


  @Override
  public void setVoltage(double volts) {
    kMotor.setControl(kVoltageControl.withOutput(volts));
  }

  @Override
  public void stop() {
    kMotor.setControl(new NeutralOut());
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {
    kMotor.setNeutralMode(enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
