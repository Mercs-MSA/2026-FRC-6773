package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract interface IndexerIO {
    @AutoLog
    public class IndexerIOInputs
    {
        public boolean isMotorConnected = false;

        public Rotation2d position = new Rotation2d();
        public Rotation2d velocityUnitsPerSec = new Rotation2d();
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
    public void updateInputs(IndexerIOInputs inputs);

    /**
     * @param volts The voltage that should be applied to the motor from -12 to 12
     */
    public void setVoltage(double volts);

    /**
     * @param goalPosition The desired angular position for the pivot to be set to. Runs using
     *     internal MotionMagic
     */
    public void setPosition(Rotation2d goalPosition);

    /**
     * Commands the hardware to stop. When using TalonFX, this commands the motors to a Neutral
     * control
     */
    public void stop();

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
    public void setGains(
        double p, double i, double d, double s, double g, double v, double a);

    /**
     * Updates the gains of the profile. Note that profiled pid control is called "MotionMagic" by
     * CTRE
     *
     * @param maxVelocity The maximum achieveable velocity of the motor in meters per second
     * @param maxAcceleration The maximum achieveable acceleration of the motor in meters per second
     *     squared
     */
    public void setMotionMagicConstraints(double maxVelocity, double maxAcceleration);

    /**
     * Enables brake or coast on the motor, only on the real motors. Useful since we usually keep them
     * on brake, but may want to set them to coast when disabled
     *
     * @param enableBrake
     */
    public void setBrakeMode(boolean enableBrake);

    /** Reset the relative encoder to 0 */
    public void resetPosition();
    
}
