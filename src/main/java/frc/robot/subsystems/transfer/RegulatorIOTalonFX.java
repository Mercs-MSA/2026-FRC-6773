package frc.robot.subsystems.transfer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.transfer.TransferConstants.TransferGains;
import frc.robot.subsystems.transfer.TransferConstants.TransferHardware;
import frc.robot.subsystems.transfer.TransferConstants.TransferTalonFXConfiguration;

public class RegulatorIOTalonFX implements TransferIO {
  private final TalonFX regulator;

  private TalonFXConfiguration motorconfig = new TalonFXConfiguration();

  // logged data for roller:
  private StatusSignal<AngularVelocity> velocityRotPerSec;
  private StatusSignal<Current> supplyAmps;
  private StatusSignal<Current> statorAmps;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Temperature> temperatureCelsius;

  private final VoltageOut kVoltageControl = new VoltageOut(0.0);

  public RegulatorIOTalonFX(
      String canbus,
      TransferHardware regulatorHardware,
      TransferTalonFXConfiguration config,
      TransferGains gains,
      double statusSignalUpdateFrequency) {
    regulator = new TalonFX(regulatorHardware.motorId(), canbus);

    motorconfig.CurrentLimits.SupplyCurrentLimitEnable = config.enableSupplyCurrentLimit();
    motorconfig.CurrentLimits.SupplyCurrentLimit = config.supplyCurrentLimitAmps();
    motorconfig.CurrentLimits.StatorCurrentLimitEnable = config.enableStatorCurrentLimit();
    motorconfig.CurrentLimits.StatorCurrentLimit = config.statorCurrentLimitAmps();
    motorconfig.Voltage.PeakForwardVoltage = config.peakForwardVoltage();
    motorconfig.Voltage.PeakReverseVoltage = config.peakReverseVoltage();
    motorconfig.MotorOutput.Inverted =
        config.invert()
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    motorconfig.MotorOutput.NeutralMode = config.neutralMode();
    motorconfig.Slot0 =
        new Slot0Configs()
            .withKP(gains.p())
            .withKI(gains.i())
            .withKD(gains.d())
            .withKV(gains.v())
            .withKS(gains.s());
    velocityRotPerSec = regulator.getVelocity();
    appliedVolts = regulator.getMotorVoltage();
    supplyAmps = regulator.getSupplyCurrent();
    statorAmps = regulator.getStatorCurrent();
    temperatureCelsius = regulator.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        statusSignalUpdateFrequency,
        velocityRotPerSec,
        appliedVolts,
        supplyAmps,
        supplyAmps,
        statorAmps,
        temperatureCelsius);

    regulator.optimizeBusUtilization(0.0, 1.0);
  }

  public RegulatorIOTalonFX(
      TransferHardware regulatorhardware,
      TransferTalonFXConfiguration config,
      TransferGains gains,
      double statusSignalUpdateFrequency) {

    // Assumes the rio is the CANBus
    this("rio", regulatorhardware, config, gains, statusSignalUpdateFrequency);
  }

  public void updateInputs(TransferIOInputs inputs) {
    inputs.isMotorConnected =
        BaseStatusSignal.refreshAll(
                velocityRotPerSec, appliedVolts, supplyAmps, statorAmps, temperatureCelsius)
            .isOK();

    inputs.velocityRotPerSec = velocityRotPerSec.getValueAsDouble();
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyAmps.getValueAsDouble();
    inputs.statorCurrentAmps = statorAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    regulator.setControl(kVoltageControl.withOutput(MathUtil.clamp(volts, -12, 12)));
  }

  @Override
  public void stop() {
    regulator.setControl(new NeutralOut());
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {
    regulator.setNeutralMode(enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setVelocity(double velocity) {
    regulator.setControl(new VelocityVoltage(velocity));
  }

  @Override
  public double getVelocity() {
    return regulator.getVelocity().getValueAsDouble();
  }
}
