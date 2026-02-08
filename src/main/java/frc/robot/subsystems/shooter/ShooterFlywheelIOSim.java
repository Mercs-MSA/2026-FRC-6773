// package frc.robot.subsystems.shooter;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.wpilibj.simulation.DCMotorSim;
// import frc.robot.subsystems.shooter.ShooterConstants.ShooterFlywheelHardware;


// public class ShooterFlywheelIOSim implements ShooterFlywheelIO {
//   private final double kLoopPeriodSec;

//   private final DCMotorSim flywheelLeft;

//   private final DCMotorSim flywheelRight;

//   private double appliedVoltage = 0.0;

//   public ShooterFlywheelIOSim(
//       double loopPeriodSec, ShooterFlywheelHardware hardware, SimulationConfiguration configuration) {
//     flywheelLeft =
//         new DCMotorSim(
//             LinearSystemId.createDCMotorSystem(
//                 configuration.motorType(), configuration.measurementStdDevs(), hardware.gearing()),
//             configuration.motorType());
//     flywheelRight =
//         new DCMotorSim(
//             LinearSystemId.createDCMotorSystem(
//                 configuration.motorType(), configuration.measurementStdDevs(), hardware.gearing()),
//             configuration.motorType());
    
//         //TODO figure out leader follower
//     kLoopPeriodSec = loopPeriodSec;
//   }

//   @Override
//   public void updateInputs(ShooterFlywheelIOInputs inputs) {
//     flywheelLeft.update(kLoopPeriodSec);
//     flywheelRight.update(kLoopPeriodSec);


//     inputs.isMotorConnected = true;

//     inputs.leftVelocityRotPerSec = flywheelLeft.getAngularVelocityRPM() / 60.0;
//     inputs.rightVelocityRotPerSec = flywheelRight.getAngularVelocityRPM() / 60.0;
//     inputs.appliedVoltage = appliedVoltage;
//     inputs.supplyCurrentAmps = 0.0;
//     inputs.statorCurrentAmps = 0.0;
//     inputs.temperatureCelsius = 0.0;
//   }

//   @Override
//   public void setVoltage(double volts) {
//     appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
//     flywheelLeft.setInputVoltage(appliedVoltage);
//   }

//   @Override
//   public void setVelocityRPS(double velocity) {
//     flywheelLeft.setAngularVelocity(velocity);
//   }

//   @Override
//   public void stop() {
//     setVoltage(0.0);
//   }
// }
