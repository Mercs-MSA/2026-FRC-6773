// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** The 2 bar intake subsystem's hardware interface */
public interface IntakeRollerIO {
  @AutoLog
  public static class IntakeRollerIOInputs {
    public boolean isMotorConnected = false;

    public double velocityRotPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;

    public boolean beambreakBroken = false;
  }

  /**
   * Write data from the hardware to the inputs object
   *
   * @param inputs The inputs object
   */
  public default void updateInputs(IntakeRollerIOInputs inputs) {}

  /**
   * @param volts The voltage that should be applied to the motor from -12 to 12
   */
  public default void setVoltage(double volts) {}

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
   * @param g
   * @param v
   * @param a
   */

  /**
   * Enables brake or coast on the motor, only on the real motors. Useful since we usually keep them
   * on brake, but may want to set them to coast when disabled
   *
   * @param enableBrake
   */
  public default void setBrakeMode(boolean enableBrake) {}
}
