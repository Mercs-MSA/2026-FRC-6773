// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterHoodIO {
  @AutoLog
  public static class ShooterHoodIOInputs {
    public boolean isMotorConnected = false;

    public Rotation2d position = new Rotation2d();
    public boolean withinRange = false;
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
  public default void updateInputs(ShooterHoodIOInputs inputs) {}

  /**
   * @param volts The voltage that should be applied to the motor from -12 to 12
   */
  public default void setVoltage(double volts) {}

  /**
   * @param goalPosition The desired angular position for the pivot to be set to. Runs using
   *     positionVoltage
   */
  public default void setPosition(Rotation2d goalPosition) {}

  /**
   * Commands the hardware to stop. When using TalonFX, this commands the motors to a Neutral
   * control
   */
  public default void stop() {}

  /**
   * Updates the gains of the feedback and feedforward
   *
   * @param p
   * @param i
   * @param d
   * @param s
   * @param v
   * @param a
   */
  public default void setGains(double p, double i, double d, double s, double v, double a) {}

  /**
   * Enables brake or coast on the motor, only on the real motors. Useful since we usually keep them
   * on brake, but may want to set them to coast when disabled
   *
   * @param enableBrake
   */
  public default void setBrakeMode(boolean enableBrake) {}

  /** Reset the relative encoder to 0 */
  public default void resetPosition() {}
}
