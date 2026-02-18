// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {
  //   public enum IntakePivotGoal {
  //     kFloorPickup(() -> Rotation2d.fromRotations(0.0)),
  //     kStow(() -> Rotation2d.fromRotations(-0.17)),
  //     kTransfer(() -> Rotation2d.fromRotations(-0.166)),
  //     kSubstationPickup(() -> Rotation2d.fromRotations(-0.153)),
  //     /** Custom setpoint that can be modified over network tables; Useful for debugging */
  //     custom(() -> Rotation2d.fromDegrees(
  //       new LoggedTunableNumber("Intake/Feedback/PivotSetpointDegrees", 0.0).get()));

  //     private Supplier<Rotation2d> goalPosition;

  //     IntakePivotGoal(Supplier<Rotation2d> goalPosition) {
  //       this.goalPosition = goalPosition;
  //     }

  //     public Rotation2d getGoalPosition() {
  //       return this.goalPosition.get();
  //     }
  //   }

  private final ShooterTurretIO turretHardware;
  private final ShooterTurretIOInputsAutoLogged turretInputs =
      new ShooterTurretIOInputsAutoLogged();

  private final ShooterFlywheelIO flywheelHardware;
  private final ShooterFlywheelIOInputsAutoLogged flywheelInputs =
      new ShooterFlywheelIOInputsAutoLogged();

  private final ShooterHoodIO hoodHardware;
  private final ShooterHoodIOInputsAutoLogged hoodInputs = new ShooterHoodIOInputsAutoLogged();

  private double minLegalAngle = Math.toRadians(-45);
  private double maxLegalAngle = Math.toRadians(45);

  private double lastGoalAngle = 0.0;

  private final LoggedNetworkNumber turret_kP =
      new LoggedNetworkNumber("Shooter/Gains/Turret_kP", ShooterConstants.turretGains.p());
  private final LoggedNetworkNumber turret_kI =
      new LoggedNetworkNumber("Shooter/Gains/Turret_kI", ShooterConstants.turretGains.i());
  private final LoggedNetworkNumber turret_kD =
      new LoggedNetworkNumber("Shooter/Gains/Turret_kD", ShooterConstants.turretGains.d());
  private final LoggedNetworkNumber turret_kS =
      new LoggedNetworkNumber("Shooter/Gains/Turret_kS", ShooterConstants.turretGains.s());
  private final LoggedNetworkNumber turret_kV =
      new LoggedNetworkNumber("Shooter/Gains/Turret_kV", ShooterConstants.turretGains.v());
  private final LoggedNetworkNumber turret_kA =
      new LoggedNetworkNumber("Shooter/Gains/Turret_kA", ShooterConstants.turretGains.a());
  private final LoggedNetworkNumber turret_maxVelocity =
      new LoggedNetworkNumber(
          "Shooter/MotionMagic/Turret_kMaxVelocity",
          ShooterConstants.turretGains.maxVelocityRotationsPerSecond());
  private final LoggedNetworkNumber turret_maxAcceleration =
      new LoggedNetworkNumber(
          "Shooter/MotionMagic/Turret_kMaxAcceleration",
          ShooterConstants.turretGains.maxAccelerationRotationsPerSecondSquared());

  private final LoggedNetworkNumber flywheel_kP =
      new LoggedNetworkNumber("Shooter/Gains/Flywheel_kP", ShooterConstants.flywheelGains.p());
  private final LoggedNetworkNumber flywheel_kI =
      new LoggedNetworkNumber("Shooter/Gains/Flywheel_kI", ShooterConstants.flywheelGains.i());
  private final LoggedNetworkNumber flywheel_kD =
      new LoggedNetworkNumber("Shooter/Gains/Flywheel_kD", ShooterConstants.flywheelGains.d());
  private final LoggedNetworkNumber flywheel_kS =
      new LoggedNetworkNumber("Shooter/Gains/Flywheel_kS", ShooterConstants.flywheelGains.s());
  private final LoggedNetworkNumber flywheel_kV =
      new LoggedNetworkNumber("Shooter/Gains/Flywheel_kV", ShooterConstants.flywheelGains.v());
  private final LoggedNetworkNumber flywheel_kA =
      new LoggedNetworkNumber("Shooter/Gains/Flywheel_kA", ShooterConstants.flywheelGains.a());
  private final LoggedNetworkNumber flywheel_kG =
      new LoggedNetworkNumber("Shooter/Gains/Flywheel_kG", ShooterConstants.flywheelGains.g());

  private final LoggedNetworkNumber flywheelVel =
      new LoggedNetworkNumber("Shooter/Flywheel/Velocity", 0);

  private final LoggedNetworkBoolean useFlyBoolean =
      new LoggedNetworkBoolean("Shooter/Flywheel/UseCustomVel", false);

  private final LoggedNetworkNumber hoodAngle = new LoggedNetworkNumber("Shooter/Hood/Angle", 0);

  private final LoggedNetworkBoolean useHoodBool =
      new LoggedNetworkBoolean("Shooter/Hood/UseCustomAngle", false);

  // private final LoggedNetworkNumber flywheel_maxVelocity =
  //     new LoggedNetworkNumber(
  //         "Shooter/MotionMagic/Flywheel_kMaxVelocity",
  //         ShooterConstants.flywheelGains.maxVelocityRotationsPerSecond());
  // private final LoggedNetworkNumber flywheel_maxAcceleration =
  //     new LoggedNetworkNumber(
  //         "Shooter/MotionMagic/Flywheel_kMaxAcceleration",
  //         ShooterConstants.flywheelGains.maxAccelerationRotationsPerSecondSquared());

  // private boolean detectedGamepiece = false;
  //   private ShooterGoal currentPivotGoal;

  /** Creates a new Shooter. */
  public Shooter(
      ShooterFlywheelIO flywheelHardwareIO,
      ShooterTurretIO turretHardwareIO,
      ShooterHoodIO hoodHardwareIO) {
    flywheelHardware = flywheelHardwareIO;
    turretHardware = turretHardwareIO;
    hoodHardware = hoodHardwareIO;

    // TODO: visualizer
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    turretHardware.updateInputs(turretInputs);
    flywheelHardware.updateInputs(flywheelInputs);
    hoodHardware.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Inputs/Turret", turretInputs);
    Logger.processInputs("Shooter/Inputs/Flywheel", flywheelInputs);
    Logger.processInputs("Shooter/Inputs/Hood", hoodInputs);

    // if (currentPivotGoal != null) {
    //   setPivotPosition(currentPivotGoal.getGoalPosition());
    //   Logger.recordOutput("Shooter/PivotGoalValue", currentPivotGoal.getGoalPosition());
    //   Logger.recordOutput("Shooter/PivotGoal", currentPivotGoal);
    // } else {
    //   Logger.recordOutput("Shooter/PivotGoal", "NONE");
    // }

    // Check if pivot is attempting to move beyond its limitations
    // if (getPivotPosition().getDegrees() > ShooterConstants.turretMaxLimit.getDegrees()
    //     && turretInputs.appliedVoltage > 0.0) {
    //   stop(false, true, false);
    // } else if (getPivotPosition().getDegrees() < ShooterConstants.turretMinLimit.getDegrees()
    //     && turretInputs.appliedVoltage < 0.0) {
    //   stop(false, true, false);
    // } else {
    //   // Do nothing if limits are not reached
    // }

    RobotState.getInstance()
        .addTurretObservation(
            new RobotState.TurretObservation(
                Timer.getTimestamp(), new Rotation2d(getTurretPosition())));

    // Log turret pose in field frame using pose transformations
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    Pose2d turretPose =
        robotPose.transformBy(
            new Transform2d(
                new Translation2d(
                    ShooterConstants.robotToTurret.getX(), ShooterConstants.robotToTurret.getY()),
                Rotation2d.fromRadians(getTurretPosition())));
    Logger.recordOutput("TurretPose", turretPose);

    if (useFlyBoolean.getAsBoolean()) {
      setFlywheelVelocityRPS(flywheelVel.getAsDouble());
    }

    if (useFlyBoolean.getAsBoolean()) {
      setHoodPosition(Rotation2d.fromDegrees(hoodAngle.getAsDouble()));
    }
  }
  //   public void setPivotGoal(IntakePivotGoal desiredGoal) {
  //     currentPivotGoal = desiredGoal;
  //   }

  //   public boolean ifStowed(){
  //     if(currentPivotGoal.equals(IntakePivotGoal.kStow)){
  //       return true;
  //     }
  //         return false;
  //   }

  public void stop(boolean stopFlywheels, boolean stopTurret, boolean stopHood) {
    if (stopFlywheels) {
      flywheelHardware.stop();
    }
    if (stopTurret) {
      //   currentPivotGoal = null;
      turretHardware.stop();
    }
    if (stopHood) {
      flywheelHardware.stop();
    }
  }

  public void setTurretVoltage(double voltage) {
    turretHardware.setVoltage(voltage);
  }

  public void setPivotPosition(Rotation2d position) {
    turretHardware.setPosition(position);
  }

  public void setFlywheelVoltage(double voltage) {
    flywheelHardware.setVoltage(voltage);
  }

  public void setFlywheelVelocityRPS(double velocity) {
    flywheelHardware.setVelocityRPS(velocity);
  }

  public void setHoodVoltage(double voltage) {
    hoodHardware.setVoltage(voltage);
  }

  public void setHoodPosition(Rotation2d position) {
    hoodHardware.setPosition(position);
  }

  @AutoLogOutput(key = "Shooter/Feedback/ErrorDegrees")
  public double getTurretErrorDegrees() {
    // if (currentPivotGoal != null && getPivotPosition() != null) {
    //   return currentPivotGoal.getGoalPosition().getDegrees() - getPivotPosition().getDegrees();
    // } else {
    //   return 0.0;
    // }
    return 0.0;
  }

  public void setTurretBrakeMode(Boolean value) {
    turretHardware.setBrakeMode(value);
  }

  public void setFlywheelBrakeMode(Boolean value) {
    flywheelHardware.setBrakeMode(value);
  }

  public void setHoodBrakeMode(Boolean value) {
    hoodHardware.setBrakeMode(value);
  }

  @AutoLogOutput(key = "Turret/Feedback/AtGoal")
  public boolean turretAtGoal() {
    return Math.abs(getTurretErrorDegrees())
        < ShooterConstants.turretPositionTolerance.getDegrees();
  }

  @AutoLogOutput(key = "Shooter/Turret/MeasuredPositionRad")
  public double getTurretPosition() {
    return turretInputs.position.getRadians();
  }

  @AutoLogOutput(key = "Shooter/Turret/VelocityRadPerSec")
  public double getTurretVelocity() {
    return turretInputs.velocityRotPerSec * 2.0 * Math.PI;
  }

  public Rotation2d getHoodPosition() {
    return hoodInputs.position;
  }

  public double[] getFlywheelVelocities() {
    return new double[] {
      flywheelInputs.leftVelocityRotPerSec, flywheelInputs.rightVelocityRotPerSec
    };
  }

  // public Command runFlywheelTrackTargetCommand() {
  // return runEnd(
  //     () ->
  //
  // setFlywheelVelocityRPS(ShooterCalculator.getInstance().getParameters().flywheelSpeed()),
  //     () -> stop(true, false, false));
  // }

  public Command runTrackTargetCommand() {
    return run(
        () -> {
          var params = ShooterCalculator.getInstance().getParameters();
          setFieldRelativeTurretTarget(params.turretAngle(), params.turretVelocity());
          setFlywheelVelocityRPS(params.flywheelSpeed());
          setHoodPosition(Rotation2d.fromRadians(params.hoodAngle()));
          // setLaunchState(LaunchState.TRACKING);
        });
  }

  public Command runHoodTrackTargetCommand() { // Todo: add velocity (similar to turret)
    return run(
        () -> {
          var params = ShooterCalculator.getInstance().getParameters();
          setHoodPosition(Rotation2d.fromRadians(params.hoodAngle()));
        });
  }

  private void setFieldRelativeTurretTarget(Rotation2d angle, double velocity) {
    double rrGoalAngle = calcRobotRelativeShooterParams(angle);
    double rrGoalVel = calcRobotRelAngVelocity(velocity);

    // Position error (proportional term) - use angle subtraction to minimize error
    double currentAngle = getTurretPosition();
    double error =
        Rotation2d.fromRadians(rrGoalAngle)
            .minus(Rotation2d.fromRadians(currentAngle))
            .getRadians();

    // // Hard bounds enforcement: clamp error to keep turret within limits
    // double clampedError =
    //     MathUtil.clamp(error, minLegalAngle - currentAngle, maxLegalAngle - currentAngle);

    // Proper PD control law:
    // P term: respond to position error (using clamped error to respect bounds)
    double pTerm = error * 1;

    // D term: damping using actual velocity (derivative of position)
    // double dTerm = -getTurretVelocity() * turret_kD.getAsDouble();

    // Feedforward term: help track moving targets (but zero out if at limit trying to exceed)
    double feedforward = rrGoalVel * 0.12;
    if ((currentAngle >= maxLegalAngle && feedforward > 0)
        || (currentAngle <= minLegalAngle && feedforward < 0)) {
      feedforward = 0;
    }

    double controlOutput = pTerm;
    double voltage = MathUtil.clamp(controlOutput, -12.0, 12.0);

    setTurretVoltage(voltage);
  }

  public double calcRobotRelAngVelocity(double fieldRelativeGoalVelocity) {
    double robotAngularVelocity = RobotState.getInstance().getFieldVelocity().omegaRadiansPerSecond;
    double robotRelativeGoalVelocity = fieldRelativeGoalVelocity - robotAngularVelocity;

    return robotRelativeGoalVelocity;
  }

  /**
   * calculates the optimized turret angle and
   *
   * @param fieldRelativeGoalAngle the goal angle relative to the field
   * @return a Rotation2D containing best turret angle
   */
  public double calcRobotRelativeShooterParams(Rotation2d fieldRelativeGoalAngle) {
    Rotation2d robotAngle = RobotState.getInstance().getRotation();

    Rotation2d robotRelativeGoalAngle = fieldRelativeGoalAngle.minus(robotAngle);

    boolean hasBestAngle = false;
    double bestAngle = getTurretPosition(); // Default to current position if no valid angle exists

    for (int i = -2; i < 3; i++) {
      double potentialSetpoint = robotRelativeGoalAngle.getRadians() + Math.PI * 2.0 * i;
      if (potentialSetpoint < minLegalAngle || potentialSetpoint > maxLegalAngle) {
        continue;
      } else {
        if (!hasBestAngle) {
          bestAngle = potentialSetpoint;
          hasBestAngle = true;
        }
        if (Math.abs(lastGoalAngle - potentialSetpoint) < Math.abs(lastGoalAngle - bestAngle)) {
          bestAngle = potentialSetpoint;
        }
      }
    }
    lastGoalAngle = bestAngle;
    // Clamp to bounds to ensure turret respects limits
    return MathUtil.clamp(bestAngle, minLegalAngle, maxLegalAngle);
  }
}
