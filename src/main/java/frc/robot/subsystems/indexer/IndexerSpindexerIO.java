package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public interface IndexerSpindexerIO {
    @AutoLog
    public static class IndexerSpindexerIOInputs {
        public boolean isMotorConnected = false;

        public AngularVelocity angularVelocity = RotationsPerSecond.of(0);
        public LinearVelocity linearVelocity = InchesPerSecond.of(0);
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
    public default void updateInputs(IndexerSpindexerIOInputs inputs) {
    }

    /**
     * @param volts The voltage that should be applied to the motor from -12 to 12
     */
    public default void setVoltage(double volts) {
    }

    /**
     * @param velocity The angular velocity the motor should reach
     */
    public default void setAngularVelocity(AngularVelocity velocity) {
    }

    /**
     * @param velocity The linear velocity the subsystem should reach
     */
    public default void setTangentialVelocity(LinearVelocity velocity) {
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

    public default void setGains(double p, double i, double d, double v, double a) {
    }

}
