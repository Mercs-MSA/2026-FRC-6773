package frc.robot.subsystems.spindexe;

import org.littletonrobotics.junction.AutoLog;

public abstract interface SpindexerIO {
  @AutoLog
  public class SpindexerIOInputs {
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
  public void updateInputs(SpindexerIOInputs inputs);

  /**
   * @param velocity The velocity that should be applied to the motor
   */
  public void setVelocity(double velocity);

  public void setVoltage(double voltage);

  /**
   * Commands the hardware to stop. When using TalonFX, this commands the motors to a Neutral
   * control
   */
  public void stop();

  /**
   * Enables brake or coast on the motor, only on the real motors. Useful since we usually keep them
   * on brake, but may want to set them to coast when disabled
   *
   * @param enableBrake
   */
  public default void setBrakeMode(boolean enableBrake) {}
  ;

  /** Returns the velocity of the motor in RPS as a double. */
  public double getVelocity();
}
