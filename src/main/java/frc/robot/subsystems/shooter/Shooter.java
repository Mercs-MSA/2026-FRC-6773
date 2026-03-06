// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.shooter.ShooterTurretCalculator.ShotData;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  public enum ShooterState {
    STOW,
    IDLE,
    IDLE_HUB,
    SHOOT_PASS_L,
    SHOOT_PASS_R,
    SHOOT_HUB,
    SHOOT_FIXED
  }

  public ShooterState shooterState = ShooterState.IDLE;

  private final ShooterTurretIO turretHardware;
  private final ShooterTurretIOInputsAutoLogged turretInputs = new ShooterTurretIOInputsAutoLogged();

  private final ShooterFlywheelIO flywheelHardware;
  private final ShooterFlywheelIOInputsAutoLogged flywheelInputs = new ShooterFlywheelIOInputsAutoLogged();

  private final ShooterHoodIO hoodHardware;
  private final ShooterHoodIOInputsAutoLogged hoodInputs = new ShooterHoodIOInputsAutoLogged();

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;

  /** Creates a new Shooter. */
  public Shooter(
      ShooterFlywheelIO flywheelHardwareIO,
      ShooterTurretIO turretHardwareIO,
      ShooterHoodIO hoodHardwareIO,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    flywheelHardware = flywheelHardwareIO;
    turretHardware = turretHardwareIO;
    hoodHardware = hoodHardwareIO;
    this.poseSupplier = poseSupplier;
    this.fieldSpeedsSupplier = fieldSpeedsSupplier;

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

    Pose2d turretBotPose = new Pose3d(poseSupplier.get()).transformBy(ShooterConstants.robotToTurret).toPose2d();
    Pose2d turretPoseOut = new Pose2d(
        turretBotPose.getX(),
        turretBotPose.getY(),
        turretBotPose.getRotation().plus(Rotation2d.fromRotations(getTurretPosition())));

    Logger.recordOutput("Shooter/Turret/TurretPose", turretPoseOut);

    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
    Pose2d robotPose = poseSupplier.get();

    ShotData calculatedShot;
    Angle azimuthAngle;
    AngularVelocity azimuthVelocity;
    Rotation2d hoodAngle;
    double flywheelVel;

    switch (shooterState) {
      case STOW:
        azimuthAngle = Angle.ofBaseUnits(0, Radians);
        azimuthVelocity = RadiansPerSecond.of(0);
        hoodAngle = Rotation2d.kZero;
        flywheelVel = 0.0;
        break;
      case IDLE:
        azimuthAngle = Angle.ofBaseUnits(0, Radians);
        azimuthVelocity = RadiansPerSecond.of(0);
        hoodAngle = Rotation2d.kZero;
        flywheelVel = 8.0;
        break;
      case IDLE_HUB:
        calculatedShot = ShooterTurretCalculator.iterativeMovingShotFromMap(robotPose, fieldSpeeds,
            FieldConstants.Hub.topCenterPoint, 2);
        azimuthAngle = ShooterTurretCalculator.calculateAzimuthAngle(
            robotPose, calculatedShot.target(), Angle.ofBaseUnits(getTurretPosition(), Rotations));
        azimuthVelocity = RadiansPerSecond.of(-fieldSpeeds.omegaRadiansPerSecond);
        hoodAngle = Rotation2d.fromDegrees(calculatedShot.getHoodAngle().in(Degrees));
        flywheelVel = 8.0;
        break;
      case SHOOT_HUB:
        calculatedShot = ShooterTurretCalculator.iterativeMovingShotFromMap(robotPose, fieldSpeeds,
            FieldConstants.Hub.topCenterPoint, 2);
        azimuthAngle = ShooterTurretCalculator.calculateAzimuthAngle(
            robotPose, calculatedShot.target(), Angle.ofBaseUnits(getTurretPosition(), Rotations));
        azimuthVelocity = RadiansPerSecond.of(-fieldSpeeds.omegaRadiansPerSecond);
        hoodAngle = Rotation2d.fromDegrees(calculatedShot.getHoodAngle().in(Degrees));
        flywheelVel = ShooterTurretCalculator.linearToAngularVelocity(
            calculatedShot.getExitVelocity(), Distance.ofBaseUnits(2, Inches))
            .in(RotationsPerSecond);
        break;
      case SHOOT_PASS_L:
        calculatedShot = ShooterTurretCalculator.iterativeMovingShotFromMap(robotPose, fieldSpeeds,
            new Translation3d(FieldConstants.LeftBump.farLeftCorner).plus(new Translation3d(0, 0, 3)), 2);
        azimuthAngle = ShooterTurretCalculator.calculateAzimuthAngle(
            robotPose, calculatedShot.target(), Angle.ofBaseUnits(getTurretPosition(), Rotations));
        azimuthVelocity = RadiansPerSecond.of(-fieldSpeeds.omegaRadiansPerSecond);
        hoodAngle = Rotation2d.fromDegrees(calculatedShot.getHoodAngle().in(Degrees));
        flywheelVel = ShooterTurretCalculator.linearToAngularVelocity(
            calculatedShot.getExitVelocity(), Distance.ofBaseUnits(2, Inches))
            .in(RotationsPerSecond);
        break;
      case SHOOT_PASS_R:
        calculatedShot = ShooterTurretCalculator.iterativeMovingShotFromMap(robotPose, fieldSpeeds,
            new Translation3d(FieldConstants.RightBump.farRightCorner).plus(new Translation3d(0, 0, 3)), 2);
        azimuthAngle = ShooterTurretCalculator.calculateAzimuthAngle(
            robotPose, calculatedShot.target(), Angle.ofBaseUnits(getTurretPosition(), Rotations));
        azimuthVelocity = RadiansPerSecond.of(-fieldSpeeds.omegaRadiansPerSecond);
        hoodAngle = Rotation2d.fromDegrees(calculatedShot.getHoodAngle().in(Degrees));
        flywheelVel = ShooterTurretCalculator.linearToAngularVelocity(
            calculatedShot.getExitVelocity(), Distance.ofBaseUnits(2, Inches))
            .in(RotationsPerSecond);
        break;
      case SHOOT_FIXED:
        calculatedShot = ShooterTurretCalculator.iterativeMovingShotFromMap(robotPose, fieldSpeeds,
            FieldConstants.Hub.topCenterPoint, 2);
        azimuthAngle = Angle.ofBaseUnits(0, Radians);
        azimuthVelocity = RadiansPerSecond.of(0);
        hoodAngle = Rotation2d.fromDegrees(calculatedShot.getHoodAngle().in(Degrees));
        flywheelVel = ShooterTurretCalculator.linearToAngularVelocity(
            calculatedShot.getExitVelocity(), Distance.ofBaseUnits(2, Inches))
            .in(RotationsPerSecond);
        break;
      default:
        azimuthAngle = Angle.ofBaseUnits(0, Radians);
        azimuthVelocity = RadiansPerSecond.of(0);
        hoodAngle = Rotation2d.kZero;
        flywheelVel = 0.0;
        break;
    }

    setTurretSetpoint(azimuthAngle, azimuthVelocity);
    setHoodPosition(hoodAngle);
    setFlywheelVelocityRPS(flywheelVel);

    Logger.recordOutput("States/ShooterState", shooterState);
  }

  public Command setShooterState(ShooterState state) {
    return Commands.runOnce(() -> {shooterState = state;}); //TODO: Make run instead of runOnce?
  }

  public void stop(boolean stopFlywheels, boolean stopTurret, boolean stopHood) {

    if (stopFlywheels) {
      flywheelHardware.stop();
    }
    if (stopTurret) {
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

  public void setTurretBrakeMode(Boolean value) {
    turretHardware.setBrakeMode(value);
  }

  public void setFlywheelBrakeMode(Boolean value) {
    flywheelHardware.setBrakeMode(value);
  }

  public void setHoodBrakeMode(Boolean value) {
    hoodHardware.setBrakeMode(value);
  }

  @AutoLogOutput(key = "Shooter/Turret/MeasuredPositionRad")
  public double getTurretPosition() {
    return turretInputs.position.getRotations();
  }

  @AutoLogOutput(key = "Shooter/Turret/VelocityRadPerSec")
  public double getTurretVelocity() {
    return turretInputs.velocityRotPerSec * 2.0 * Math.PI;
  }

  @AutoLogOutput(key = "Shooter/Hood/HoodPosition")
  public Rotation2d getHoodPosition() {
    return hoodInputs.position;
  }

  @AutoLogOutput(key = "Shooter/Hood/HoodPosition")
  public AngularVelocity[] getFlywheelVelocities() {
    return new AngularVelocity[] {
        flywheelInputs.leftVelocity, flywheelInputs.rightVelocity
    };
  }

  public void setTurretSetpoint(Angle angle, AngularVelocity angvel) {
    turretHardware.setTurretSetpoint(angle, angvel);
  }
}
