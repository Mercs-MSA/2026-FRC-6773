package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelGains;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelMotorConfiguration;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterFlywheelHardware;

public class ShooterFlywheelIOTalonFX implements ShooterFlywheelIO {
  private final TalonFX flywheelMotorLeft;
  private final TalonFX flywheelMotorRight;

  private TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

  // logged data for roller:
  private StatusSignal<AngularVelocity> leftVelocityRotPerSec;
  private StatusSignal<AngularVelocity> rightVelocityRotPerSec;
  private StatusSignal<Current> supplyAmps;
  private StatusSignal<Current> statorAmps;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Temperature> temperatureCelsius;

  private final VelocityVoltage kvelocityVoltage = new VelocityVoltage(0.0);

  public ShooterFlywheelIOTalonFX(
      String canbus,
      ShooterFlywheelHardware hardware,
      FlywheelMotorConfiguration configuration,
      FlywheelGains gains,
      double statusSignalUpdateFrequency) {
    flywheelMotorLeft = new TalonFX(hardware.flyWheelMotorLeftId());
    flywheelMotorRight = new TalonFX(hardware.flyWheelMotorRightId());

    flywheelMotorRight.setControl(
        new Follower(hardware.flyWheelMotorLeftId(), MotorAlignmentValue.Opposed));

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

    leftVelocityRotPerSec = flywheelMotorLeft.getVelocity();
    rightVelocityRotPerSec = flywheelMotorRight.getVelocity();
    appliedVolts = flywheelMotorLeft.getMotorVoltage();
    supplyAmps = flywheelMotorLeft.getSupplyCurrent();
    statorAmps = flywheelMotorLeft.getStatorCurrent();
    temperatureCelsius = flywheelMotorLeft.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        statusSignalUpdateFrequency,
        leftVelocityRotPerSec,
        rightVelocityRotPerSec,
        appliedVolts,
        supplyAmps,
        supplyAmps,
        statorAmps,
        temperatureCelsius);

    flywheelMotorLeft.optimizeBusUtilization(0.0, 1.0);
    flywheelMotorRight.optimizeBusUtilization(0.0, 1.0);
    flywheelMotorLeft.getConfigurator().apply(motorConfiguration, 1);
    flywheelMotorRight.getConfigurator().apply(motorConfiguration, 1);
  }

  public ShooterFlywheelIOTalonFX(
      ShooterFlywheelHardware hardware,
      FlywheelMotorConfiguration configuration,
      FlywheelGains gains,
      double statusSignalUpdateFrequency) {

    // Assumes the rio is the CANBus
    this("rio", hardware, configuration, gains, statusSignalUpdateFrequency);
  }

  public void updateInputs(ShooterFlywheelIOInputs inputs) {
    inputs.isMotorConnected =
        BaseStatusSignal.refreshAll(
                leftVelocityRotPerSec,
                rightVelocityRotPerSec,
                appliedVolts,
                supplyAmps,
                statorAmps,
                temperatureCelsius)
            .isOK();

    inputs.leftVelocityRotPerSec = leftVelocityRotPerSec.getValueAsDouble();
    inputs.rightVelocityRotPerSec = rightVelocityRotPerSec.getValueAsDouble();
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyAmps.getValueAsDouble();
    inputs.statorCurrentAmps = statorAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
  }

  // @Override
  // public void setVoltage(double volts) {
  //   flywheelMotorLeft.setControl(kvelocityVoltage.withVelocity(volts));
  // }

  @Override
  public void setVelocityRPS(double velocity) {
    flywheelMotorLeft.setControl(kvelocityVoltage.withVelocity(velocity));
  }
  /*Some info from the docs:
  * Motion Magic® Velocity produces a motion profile in real-time while attempting to honor the specified Acceleration and (optional) Jerk. This control mode does not use the CruiseVelocity, Expo_kV, or Expo_kA configs.

   If the specified acceleration is zero, the Acceleration under Motion Magic® configuration parameter is used instead. This allows for runtime adjustment of acceleration for advanced users. Jerk is also specified in the Motion Magic® persistent configuration values. If Jerk is set to zero, Motion Magic® will produce a trapezoidal acceleration profile.

   Target velocity can also be changed on-the-fly and Motion Magic® will do its best to adjust the profile. This control mode is voltage-based, so relevant closed-loop gains will use Volts for the numerator.
  */

  @Override
  public void setGains(double p, double i, double d, double v, double a) {
    var slotConfiguration = new Slot0Configs();

    slotConfiguration.kP = p;
    slotConfiguration.kI = i;
    slotConfiguration.kD = d;
    slotConfiguration.kV = v;
    slotConfiguration.kA = a;

    flywheelMotorLeft.getConfigurator().apply((slotConfiguration));
    flywheelMotorRight.getConfigurator().apply((slotConfiguration));
  }

  @Override
  public void stop() {
    flywheelMotorLeft.setControl(new NeutralOut());
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {
    flywheelMotorLeft.setNeutralMode(enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public double getStatorCurrent()
  {
    return flywheelMotorLeft.getStatorCurrent().getValueAsDouble();
  }
}
