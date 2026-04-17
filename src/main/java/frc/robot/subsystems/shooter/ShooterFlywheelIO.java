// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterFlywheelIO {
  @AutoLog
  public static class ShooterFlywheelIOInputs {
    public boolean isMotorConnected = false;

    public boolean hasCommand = false;

    public AngularVelocity leftVelocity = RotationsPerSecond.of(0);
    public AngularVelocity rightVelocity = RotationsPerSecond.of(0);
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /**
   * Write data from the hardware to the inputs object
   *
   * @param inputs The inputs object
   */
  public default void updateInputs(ShooterFlywheelIOInputs inputs) {}

  /**
   * @param volts The voltage that should be applied to the motor from -12 to 12
   */
  public default void setVoltage(double volts) {}

  public default void setVelocityRPS(double velocity) {}

  public default void setGains(double p, double i, double d, double v, double a) {}

  public default void coastOut() {}
  /**
   * Commands the hardware to stop. When using TalonFX, this commands the motors to a Neutral
   * control
   */
  public default void stop() {}

  /**
   * Enables brake or coast on the motor, only on the real motors. Useful since we usually keep them
   * on brake, but may want to set them to coast when disabled
   *
   * @param enableBrake
   */
  public default void setBrakeMode(boolean enableBrake) {}
}
