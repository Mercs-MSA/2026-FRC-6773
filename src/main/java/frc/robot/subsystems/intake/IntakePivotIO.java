package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakePivotIO {
    @AutoLog
    public static class IntakePivotIOInputs {
        public boolean isMotorConnected = false;

        public double velocityRotPerSec = 0.0;
        public Rotation2d position = Rotation2d.kZero;
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
    public default void updateInputs(IntakePivotIOInputs inputs) {
    }

    /**
     * @param volts The voltage that should be applied to the motor from -12 to 12
     */
    public default void setVoltage(double volts) {
    }

    /**
     * @param position The position the mechanism should reach
     */
    public default void setPosition(Rotation2d position) {
    }

    /**
     * Commands the hardware to stop. When using TalonFX, this commands the motors
     * to a Neutral
     * control
     */
    public default void stop() {
    }

    /**
     * Enables brake or coast on the motor, only on the real motors. Useful since we
     * usually keep them
     * on brake, but may want to set them to coast when disabled
     *
     * @param enableBrake
     */
    public default void setBrakeMode(boolean enableBrake) {
    }

}
