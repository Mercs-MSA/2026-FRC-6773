package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeConstants.RollerHardware;
import frc.robot.subsystems.intake.IntakeConstants.RollerTalonFXConfiguration;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final TalonFX leftRollerMotor;
  private final TalonFX rightRollerMotor;

  private NeutralModeValue currentMode = NeutralModeValue.Brake;

  private TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

  private final VoltageOut motorControl = new VoltageOut(0.0);

  private StatusSignal<AngularVelocity> velocityRotPerSec;
  private StatusSignal<Current> supplyCurrentAmps;
  private StatusSignal<Current> statorCurrentAmps;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Temperature> temperatureCelsius;

  public IntakeRollerIOTalonFX(
      String canbus,
      RollerHardware hardware,
      RollerTalonFXConfiguration configuration,
      double statusSignalUpdateFrequency) {
    leftRollerMotor = new TalonFX(hardware.leftRollerID(), canbus);
    rightRollerMotor = new TalonFX(hardware.rightRollerID(), canbus);

    leftRollerMotor.setControl(new Follower(hardware.rightRollerID(), MotorAlignmentValue.Opposed));

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

    velocityRotPerSec = rightRollerMotor.getVelocity();
    appliedVolts = rightRollerMotor.getMotorVoltage();
    supplyCurrentAmps = rightRollerMotor.getSupplyCurrent();
    statorCurrentAmps = rightRollerMotor.getStatorCurrent();
    temperatureCelsius = rightRollerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        statusSignalUpdateFrequency,
        velocityRotPerSec,
        appliedVolts,
        supplyCurrentAmps,
        statorCurrentAmps,
        temperatureCelsius);

    leftRollerMotor.getConfigurator().apply(motorConfiguration, 1);
    rightRollerMotor.getConfigurator().apply(motorConfiguration, 1);
  }

  public IntakeRollerIOTalonFX(
      RollerHardware hardware,
      RollerTalonFXConfiguration config,
      double statusSignalUpdateFrequency) {
    // Assumes the rio is the CANBus
    this("rio", hardware, config, statusSignalUpdateFrequency);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
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
    rightRollerMotor.setControl(motorControl.withOutput(voltage));
  }

  @Override
  public void stop() {
    rightRollerMotor.setControl(new NeutralOut());
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {
    NeutralModeValue newMode = enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    if (currentMode != newMode) {
      rightRollerMotor.setNeutralMode(newMode);
      currentMode = newMode;
    }
  }
}
