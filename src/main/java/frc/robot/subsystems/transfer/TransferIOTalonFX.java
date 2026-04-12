package frc.robot.subsystems.transfer;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.transfer.TransferConstants.TransferGains;
import frc.robot.subsystems.transfer.TransferConstants.TransferHardware;
import frc.robot.subsystems.transfer.TransferConstants.TransferTalonFXConfiguration;

public class TransferIOTalonFX implements TransferIO {
  private final TalonFX transferMotor;

  private TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

  private double wheelRadius;

  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Current> supplyAmps;
  private StatusSignal<Current> statorAmps;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Temperature> temperatureCelsius;

  private VelocityVoltage motorControl = new VelocityVoltage(0);

  public TransferIOTalonFX(
      String canbus,
      TransferHardware hardware,
      TransferGains gains,
      TransferTalonFXConfiguration configuration,
      double statusSignalUpdateFrequency) {
    transferMotor = new TalonFX(hardware.transferID(), canbus);

    motorConfiguration.Slot0.kP = gains.p();
    motorConfiguration.Slot0.kI = gains.i();
    motorConfiguration.Slot0.kD = gains.d();
    motorConfiguration.Slot0.kV = gains.v();
    motorConfiguration.Slot0.kA = gains.a();

    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable =
        configuration.enableSupplyCurrentLimit();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit = configuration.supplyCurrentLimitAmps();
    motorConfiguration.CurrentLimits.StatorCurrentLimitEnable =
        configuration.enableStatorCurrentLimit();
    motorConfiguration.CurrentLimits.StatorCurrentLimit = configuration.statorCurrentLimitAmps();
    motorConfiguration.Voltage.PeakForwardVoltage = configuration.peakForwardVoltage();
    motorConfiguration.Voltage.PeakReverseVoltage = configuration.peakReverseVoltage();
    motorConfiguration.MotorOutput.Inverted =
        configuration.invert()
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    motorConfiguration.MotorOutput.NeutralMode = configuration.neutralMode();

    velocity = transferMotor.getVelocity();
    supplyAmps = transferMotor.getSupplyCurrent();
    statorAmps = transferMotor.getStatorCurrent();
    appliedVolts = transferMotor.getMotorVoltage();
    temperatureCelsius = transferMotor.getDeviceTemp();

    wheelRadius = hardware.wheelRadIn();

    BaseStatusSignal.setUpdateFrequencyForAll(
        statusSignalUpdateFrequency,
        velocity,
        appliedVolts,
        supplyAmps,
        supplyAmps,
        statorAmps,
        temperatureCelsius);

    transferMotor.optimizeBusUtilization(0.0, 1.0);
    transferMotor.getConfigurator().apply(motorConfiguration, 1);
  }

  public TransferIOTalonFX(
      TransferHardware hardware,
      TransferGains gains,
      TransferTalonFXConfiguration configuration,
      double statusSignalUpdateFrequency) {

    // Assumes the rio is the CANBus
    this("rio", hardware, gains, configuration, statusSignalUpdateFrequency);
  }

  @Override
  public void updateInputs(TransferIOInputs inputs) {
    inputs.isMotorConnected =
        BaseStatusSignal.refreshAll(
                velocity, appliedVolts, supplyAmps, statorAmps, temperatureCelsius)
            .isOK();

    inputs.angularVelocity = velocity.getValue();
    inputs.linearVelocity =
        InchesPerSecond.of(velocity.getValue().in(RotationsPerSecond) * wheelRadius * 2 * Math.PI);
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyAmps.getValueAsDouble();
    inputs.statorCurrentAmps = statorAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    transferMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    transferMotor.setControl(motorControl.withVelocity(velocity));
  }

  @Override
  public void setTangentialVelocity(LinearVelocity velocity) {
    setAngularVelocity(
        RotationsPerSecond.of(velocity.in(InchesPerSecond) / (2 * wheelRadius * Math.PI)));
  }

  @Override
  public void stop() {
    transferMotor.setControl(new NeutralOut());
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {
    transferMotor.setNeutralMode(enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setGains(double p, double i, double d, double v, double a) {
    var slotConfiguration = new Slot0Configs();

    slotConfiguration.kP = p;
    slotConfiguration.kI = i;
    slotConfiguration.kD = d;
    slotConfiguration.kV = v;
    slotConfiguration.kA = a;

    transferMotor.getConfigurator().apply((slotConfiguration));
  }
}
