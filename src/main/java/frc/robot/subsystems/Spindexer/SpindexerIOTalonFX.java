package frc.robot.subsystems.Spindexer;

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
import frc.robot.subsystems.Spindexer.SpindexerConstants.SpindexerGains;
import frc.robot.subsystems.Spindexer.SpindexerConstants.SpindexerHardware;
import frc.robot.subsystems.Spindexer.SpindexerConstants.SpindexerTalonFXConfiguration;
import frc.robot.subsystems.Spindexer.SpindexerIO.SpindexerIOInputs;

public class SpindexerIOTalonFX implements SpindexerIO {
  private final TalonFX kMotor;

  private TalonFXConfiguration motorconfig = new TalonFXConfiguration();

  // logged data for roller:
  private StatusSignal<AngularVelocity> velocityRotPerSec;
  private StatusSignal<Current> supplyAmps;
  private StatusSignal<Current> statorAmps;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Temperature> temperatureCelsius;

  private final VoltageOut kVoltageControl = new VoltageOut(0.0);

  public SpindexerIOTalonFX(
      String canbus,
      SpindexerHardware hardware,
      SpindexerTalonFXConfiguration config,
      SpindexerGains gains,
      double statusSignalUpdateFrequency) {
    kMotor = new TalonFX(hardware.motorId(), canbus);

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
    velocityRotPerSec = kMotor.getVelocity();
    appliedVolts = kMotor.getMotorVoltage();
    supplyAmps = kMotor.getSupplyCurrent();
    statorAmps = kMotor.getStatorCurrent();
    temperatureCelsius = kMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        statusSignalUpdateFrequency,
        velocityRotPerSec,
        appliedVolts,
        supplyAmps,
        supplyAmps,
        statorAmps,
        temperatureCelsius);

    kMotor.optimizeBusUtilization(0.0, 1.0); // TODO: What is this?
  }

  public SpindexerIOTalonFX(
      SpindexerHardware hardware,
      SpindexerTalonFXConfiguration config,
      SpindexerGains gains,
      double statusSignalUpdateFrequency) {

    // Assumes the rio is the CANBus
    this("rio", hardware, config, gains, statusSignalUpdateFrequency);
  }

  public void updateInputs(SpindexerIOInputs inputs) {
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
    kMotor.setControl(kVoltageControl.withOutput(MathUtil.clamp(volts, -12, 12)));
  }

  @Override
  public void stop() {
    kMotor.setControl(new NeutralOut());
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {
    kMotor.setNeutralMode(enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setVelocity(double velocity) {
    kMotor.setControl(new VelocityVoltage(velocity));
  }

  @Override
  public double getVelocity() {
    return kMotor.getVelocity().getValueAsDouble();
  }
}
