package frc.robot.subsystems.climb;

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
import frc.robot.subsystems.climb.ClimbConstants.ClimbHardware;
import frc.robot.subsystems.climb.ClimbConstants.ClimbTalonFXConfiguration;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX climbMotor;

  private NeutralModeValue currentMode = NeutralModeValue.Brake;

  private TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

  private final VoltageOut motorControl = new VoltageOut(0.0);

  private StatusSignal<AngularVelocity> velocityRotPerSec;
  private StatusSignal<Current> supplyCurrentAmps;
  private StatusSignal<Current> statorCurrentAmps;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Temperature> temperatureCelsius;

  public ClimbIOTalonFX(
      String canbus,
      ClimbHardware hardware,
      ClimbTalonFXConfiguration configuration,
      double statusSignalUpdateFrequency) {
    climbMotor = new TalonFX(hardware.ClimbID(), canbus);

    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable =
        configuration.enableSupplyCurrentLimit();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit = configuration.supplyCurrentLimitAmps();
    motorConfiguration.CurrentLimits.StatorCurrentLimitEnable =
        configuration.enableStatorCurrentLimit();
    motorConfiguration.CurrentLimits.StatorCurrentLimit = configuration.statorCurrentLimitAmps();
    motorConfiguration.Voltage.PeakForwardVoltage = configuration.peakForwardVoltage();
    motorConfiguration.Voltage.PeakReverseVoltage = configuration.peakReverseVoltage();

    motorConfiguration.MotorOutput.NeutralMode = configuration.neutralMode();
    motorConfiguration.MotorOutput.Inverted =
        configuration.invert()
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    motorConfiguration.Feedback.SensorToMechanismRatio = hardware.gearing();
    motorConfiguration.Feedback.RotorToSensorRatio = 1.0;

    velocityRotPerSec = climbMotor.getVelocity();
    appliedVolts = climbMotor.getMotorVoltage();
    supplyCurrentAmps = climbMotor.getSupplyCurrent();
    statorCurrentAmps = climbMotor.getStatorCurrent();
    temperatureCelsius = climbMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        statusSignalUpdateFrequency,
        velocityRotPerSec,
        appliedVolts,
        supplyCurrentAmps,
        statorCurrentAmps,
        temperatureCelsius);

    climbMotor.getConfigurator().apply(motorConfiguration, 1);
  }

  public ClimbIOTalonFX(
      ClimbHardware hardware,
      ClimbTalonFXConfiguration config,
      double statusSignalUpdateFrequency) {
    // Assumes the rio is the CANBus
    this("rio", hardware, config, statusSignalUpdateFrequency);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.isMotorConnected =
        BaseStatusSignal.refreshAll(
                velocityRotPerSec,
                appliedVolts,
                supplyCurrentAmps,
                supplyCurrentAmps,
                statorCurrentAmps,
                temperatureCelsius)
            .isOK();

    inputs.velocityRotPerSec = velocityRotPerSec.getValueAsDouble();
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    climbMotor.setControl(motorControl.withOutput(voltage));
  }

  @Override
  public void stop() {
    climbMotor.setControl(new NeutralOut());
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {
    NeutralModeValue newMode = enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    if (currentMode != newMode) {
      climbMotor.setNeutralMode(newMode);
      currentMode = newMode;
    }
  }
}
