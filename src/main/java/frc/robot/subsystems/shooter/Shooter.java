// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.shooter.ShooterTurretCalculator.ShotData;
import frc.robot.util.ZoneUtil;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {

  public enum ShooterState {
    STOW,
    IDLE_FIXED,
    IDLE_HUB,
    IDLE_L,
    IDLE_R,
    SHOOT_PASS_L,
    SHOOT_PASS_R,
    SHOOT_HUB,
    SHOOT_FIXED,
    SHOOT_TRENCH_LOCK,
    TRENCH_MANUAL,
  }

  public final LoggedNetworkNumber hoodAngleCust = new LoggedNetworkNumber("Shooter/HoodAngle", 0);
  public final LoggedNetworkNumber flywheelVelCust =
      new LoggedNetworkNumber("Shooter/FlywheelVel", 0);
  public final LoggedNetworkBoolean useCustom;
  public final LoggedNetworkBoolean useManual;
  public final LoggedNetworkBoolean useBiases;

  public final LoggedNetworkBoolean fixedShoot =
      new LoggedNetworkBoolean("Shooter/Fixed Turret", false);

  public ShooterState shooterState = ShooterState.STOW;

  private final ShooterTurretIO turretHardware;
  private final ShooterTurretIOInputsAutoLogged turretInputs =
      new ShooterTurretIOInputsAutoLogged();

  private final ShooterFlywheelIO flywheelHardware;
  private final ShooterFlywheelIOInputsAutoLogged flywheelInputs =
      new ShooterFlywheelIOInputsAutoLogged();

  private final ShooterHoodIO hoodHardware;
  private final ShooterHoodIOInputsAutoLogged hoodInputs = new ShooterHoodIOInputsAutoLogged();

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;

  public double turretBias = 0.0;
  public double flywheelBias = 0.0;
  public double hoodBias = 0.0;

  Trigger allianceZoneTrigger;
  Trigger leftPassTrigger;
  Trigger rightPassTrigger;

  BooleanSupplier fixedShooter;

  private final Supplier<Alliance> isBlue;

  private Angle turretSetPoint;

  /** Creates a new Shooter. */
  public Shooter(
      ShooterFlywheelIO flywheelHardwareIO,
      ShooterTurretIO turretHardwareIO,
      ShooterHoodIO hoodHardwareIO,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier) {

    useCustom = new LoggedNetworkBoolean("Shooter/Use Customs", false);
    useBiases = new LoggedNetworkBoolean("Shooter/Use Biases", false);
    useManual = new LoggedNetworkBoolean("Shooter/Use Manual", false);
    flywheelHardware = flywheelHardwareIO;
    turretHardware = turretHardwareIO;
    hoodHardware = hoodHardwareIO;
    this.poseSupplier = poseSupplier;
    this.fieldSpeedsSupplier = fieldSpeedsSupplier;
    fixedShooter = () -> fixedShoot.getAsBoolean();
    Trigger fixShooter = new Trigger(fixedShooter);
    fixShooter.onTrue(Commands.runOnce(() -> setShooterState(ShooterState.IDLE_FIXED)));
    fixShooter.onFalse(Commands.runOnce(() -> setShooterState(ShooterState.IDLE_HUB)));

    isBlue =
        () -> {
          return DriverStation.getAlliance().get();
        };

    allianceZoneTrigger =
        isBlue.get().equals(Alliance.Red)
            ? ZoneUtil.RED_ALLIANCE_ZONES.contains(poseSupplier)
            : ZoneUtil.BLUE_ALLIANCE_ZONES.contains(poseSupplier);
    rightPassTrigger =
        isBlue.get().equals(Alliance.Red)
            ? ZoneUtil.RED_RIGHT_PASS_ZONE.contains(poseSupplier)
            : ZoneUtil.BLUE_RIGHT_PASS_ZONE.contains(poseSupplier);
    leftPassTrigger =
        isBlue.get().equals(Alliance.Red)
            ? ZoneUtil.RED_LEFT_PASS_ZONE.contains(poseSupplier)
            : ZoneUtil.BLUE_LEFT_PASS_ZONE.contains(poseSupplier);

    allianceZoneTrigger.onTrue(
        Commands.runOnce(
            () -> {
              if (fixedShooter.getAsBoolean()) {
                setShooterState(ShooterState.IDLE_FIXED);
              } else {
                setShooterState(ShooterState.IDLE_HUB);
              }
            }));
    rightPassTrigger.onTrue(
        Commands.runOnce(
            () -> {
              if (fixedShooter.getAsBoolean()) {
                setShooterState(ShooterState.IDLE_FIXED);
              } else {
                setShooterState(ShooterState.IDLE_R);
              }
            }));
    leftPassTrigger.onTrue(
        Commands.runOnce(
            () -> {
              if (fixedShooter.getAsBoolean()) {
                setShooterState(ShooterState.IDLE_FIXED);
              } else {
                setShooterState(ShooterState.IDLE_L);
              }
            }));
    // TODO: visualizer
    turretSetPoint = Angle.ofBaseUnits(0, Rotations);
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
    Logger.recordOutput("IsBlueSupplier", isBlue.get().toString());

    Pose2d turretBotPose =
        new Pose3d(poseSupplier.get()).transformBy(ShooterConstants.robotToTurret).toPose2d();
    Pose2d turretPoseOut =
        new Pose2d(
            turretBotPose.getX(),
            turretBotPose.getY(),
            turretBotPose
                .getRotation()
                .plus(Rotation2d.fromRotations(getTurretPosition()))
                .plus(Rotation2d.kCCW_Pi_2));

    Logger.recordOutput("Shooter/Turret/TurretPose", turretPoseOut);

    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
    Pose2d robotPose = poseSupplier.get();

    ShotData calculatedShot;
    Angle azimuthAngle;
    AngularVelocity azimuthVelocity;
    Rotation2d hoodAngle;
    double flywheelVel;
    if (useManual.getAsBoolean()) {
      shooterState = ShooterState.TRENCH_MANUAL;
    }
    switch (shooterState) {
      case STOW:
        azimuthAngle = Angle.ofBaseUnits(0, Radians);
        azimuthVelocity = RadiansPerSecond.of(0);
        hoodAngle = Rotation2d.kZero;
        flywheelVel = 0.0;
        break;
      case IDLE_FIXED:
        azimuthAngle = Rotations.of(getTurretPosition());
        azimuthVelocity = RadiansPerSecond.of(0);
        hoodAngle = getHoodPosition();
        flywheelVel = 8.0;
        break;
      case IDLE_L:
        calculatedShot =
            ShooterTurretCalculator.iterativeMovingShotFromMap(
                robotPose,
                fieldSpeeds,
                FieldConstants.Hub.topCenterPoint.plus(new Translation3d(-2.5, 2.5, -1.2)),
                2,
                true);
        azimuthAngle =
            ShooterTurretCalculator.calculateAzimuthAngle(
                robotPose,
                calculatedShot.target(),
                Angle.ofBaseUnits(getTurretPosition(), Rotations));
        azimuthVelocity =
            ShooterTurretCalculator.getTurretAzimuthVelocity(
                robotPose, calculatedShot.target(), fieldSpeeds);
        hoodAngle = Rotation2d.fromDegrees(calculatedShot.getHoodAngle().in(Degrees));
        flywheelVel = 8.0;
        break;
      case IDLE_R:
        calculatedShot =
            ShooterTurretCalculator.iterativeMovingShotFromMap(
                robotPose,
                fieldSpeeds,
                FieldConstants.Hub.topCenterPoint.plus(new Translation3d(-2.5, -2.5, -1.2)),
                2,
                true);
        azimuthAngle =
            ShooterTurretCalculator.calculateAzimuthAngle(
                robotPose,
                calculatedShot.target(),
                Angle.ofBaseUnits(getTurretPosition(), Rotations));
        azimuthVelocity =
            ShooterTurretCalculator.getTurretAzimuthVelocity(
                robotPose, calculatedShot.target(), fieldSpeeds);
        hoodAngle = Rotation2d.fromDegrees(calculatedShot.getHoodAngle().in(Degrees));
        flywheelVel = 8.0;
        break;
      case IDLE_HUB:
        calculatedShot =
            ShooterTurretCalculator.iterativeMovingShotFromMap(
                robotPose, fieldSpeeds, FieldConstants.Hub.topCenterPoint, 2, false);
        azimuthAngle =
            ShooterTurretCalculator.calculateAzimuthAngle(
                robotPose,
                calculatedShot.target(),
                Angle.ofBaseUnits(getTurretPosition(), Rotations));
        azimuthVelocity =
            ShooterTurretCalculator.getTurretAzimuthVelocity(
                robotPose, calculatedShot.target(), fieldSpeeds);
        hoodAngle = Rotation2d.fromDegrees(calculatedShot.getHoodAngle().in(Degrees));
        flywheelVel = 8.0;
        break;
      case SHOOT_HUB:
        calculatedShot =
            ShooterTurretCalculator.iterativeMovingShotFromMap(
                robotPose, fieldSpeeds, FieldConstants.Hub.topCenterPoint, 2, false);
        azimuthAngle =
            ShooterTurretCalculator.calculateAzimuthAngle(
                robotPose,
                calculatedShot.target(),
                Angle.ofBaseUnits(getTurretPosition(), Rotations));
        azimuthVelocity =
            ShooterTurretCalculator.getTurretAzimuthVelocity(
                robotPose, calculatedShot.target(), fieldSpeeds);
        hoodAngle = Rotation2d.fromDegrees(calculatedShot.getHoodAngle().in(Degrees));
        flywheelVel =
            ShooterTurretCalculator.linearToAngularVelocity(
                    calculatedShot.getExitVelocity(), Distance.ofBaseUnits(2, Inches))
                .in(RotationsPerSecond);
        break;
      case SHOOT_PASS_L:
        calculatedShot =
            ShooterTurretCalculator.iterativeMovingShotFromMap(
                robotPose,
                fieldSpeeds,
                FieldConstants.Hub.topCenterPoint.plus(new Translation3d(-2.5, 2.5, -1.2)),
                2,
                true);
        azimuthAngle =
            ShooterTurretCalculator.calculateAzimuthAngle(
                robotPose,
                calculatedShot.target(),
                Angle.ofBaseUnits(getTurretPosition(), Rotations));
        azimuthVelocity =
            ShooterTurretCalculator.getTurretAzimuthVelocity(
                robotPose, calculatedShot.target(), fieldSpeeds);
        hoodAngle = Rotation2d.fromDegrees(calculatedShot.getHoodAngle().in(Degrees));
        flywheelVel =
            ShooterTurretCalculator.linearToAngularVelocity(
                    calculatedShot.getExitVelocity(), Distance.ofBaseUnits(2, Inches))
                .in(RotationsPerSecond);
        break;
      case SHOOT_PASS_R:
        calculatedShot =
            ShooterTurretCalculator.iterativeMovingShotFromMap(
                robotPose,
                fieldSpeeds,
                FieldConstants.Hub.topCenterPoint.plus(new Translation3d(-2.5, -2.5, -1.2)),
                2,
                true);
        azimuthAngle =
            ShooterTurretCalculator.calculateAzimuthAngle(
                robotPose,
                calculatedShot.target(),
                Angle.ofBaseUnits(getTurretPosition(), Rotations));
        azimuthVelocity =
            ShooterTurretCalculator.getTurretAzimuthVelocity(
                robotPose, calculatedShot.target(), fieldSpeeds);
        hoodAngle = Rotation2d.fromDegrees(calculatedShot.getHoodAngle().in(Degrees));
        flywheelVel =
            ShooterTurretCalculator.linearToAngularVelocity(
                    calculatedShot.getExitVelocity(), Distance.ofBaseUnits(2, Inches))
                .in(RotationsPerSecond);
        break;
      case SHOOT_FIXED:
        calculatedShot =
            ShooterTurretCalculator.iterativeMovingShotFromMap(
                robotPose, fieldSpeeds, FieldConstants.Hub.topCenterPoint, 2, false);
        azimuthAngle = Rotations.of(getTurretPosition());
        azimuthVelocity = RadiansPerSecond.of(0);
        hoodAngle = Rotation2d.fromDegrees(calculatedShot.getHoodAngle().in(Degrees));
        flywheelVel =
            ShooterTurretCalculator.linearToAngularVelocity(
                    calculatedShot.getExitVelocity(), Distance.ofBaseUnits(2, Inches))
                .in(RotationsPerSecond);
        break;
      case TRENCH_MANUAL:
        azimuthAngle = Rotations.of(getTurretPosition());
        azimuthVelocity = RadiansPerSecond.of(-fieldSpeeds.omegaRadiansPerSecond);
        hoodAngle = Rotation2d.fromDegrees(20);
        flywheelVel = 58;
        break;
      default:
        azimuthAngle = Angle.ofBaseUnits(0, Radians);
        azimuthVelocity = RadiansPerSecond.of(0);
        hoodAngle = Rotation2d.kZero;
        flywheelVel = 0.0;
        break;
    }

    turretSetPoint = azimuthAngle;
    if (useBiases.getAsBoolean()) {
      if (shooterState != ShooterState.TRENCH_MANUAL)
        setTurretSetpoint(
            azimuthAngle.plus(Angle.ofBaseUnits(turretBias, Rotations)), azimuthVelocity);
      if (!useCustom.getAsBoolean()) {
        double value = hoodAngle.getRotations() + hoodBias;
        setHoodPosition(new Rotation2d(MathUtil.clamp(value, 0, 20d / 360d)));
        if (shooterState == ShooterState.STOW
            || shooterState == ShooterState.IDLE_FIXED
            || shooterState == ShooterState.IDLE_HUB
            || shooterState == ShooterState.IDLE_L
            || shooterState == ShooterState.IDLE_L
            || shooterState == ShooterState.IDLE_R) {
          flywheelHardware.coastOut();
        } else {
          setFlywheelVelocityRPS(flywheelVel + flywheelBias);
        }
      } else {
        setHoodPosition(new Rotation2d(hoodAngleCust.getAsDouble()));
        setFlywheelVelocityRPS(flywheelVelCust.getAsDouble());
      }
    } else {
      if (shooterState != ShooterState.TRENCH_MANUAL)
        setTurretSetpoint(azimuthAngle, azimuthVelocity);

      if (!useCustom.getAsBoolean()) {
        if (shooterState == ShooterState.STOW
            || shooterState == ShooterState.IDLE_FIXED
            || shooterState == ShooterState.IDLE_HUB
            || shooterState == ShooterState.IDLE_L
            || shooterState == ShooterState.IDLE_L
            || shooterState == ShooterState.IDLE_R) {
          flywheelHardware.coastOut();
        } else {
          setFlywheelVelocityRPS(flywheelVel);
        }
        setHoodPosition(hoodAngle);
      } else {
        setHoodPosition(new Rotation2d(hoodAngleCust.getAsDouble()));
        setFlywheelVelocityRPS(flywheelVelCust.getAsDouble());
      }
    }
    Logger.recordOutput("FlywheelDebug/azimuthAngle", azimuthAngle);
    Logger.recordOutput("FlywheelDebug/targetFlywheelVelRPS", flywheelVel);
    Logger.recordOutput(
        "FlywheelDebug/flywheelRPS", getFlywheelVelocities()[0].in(RotationsPerSecond));
    Logger.recordOutput("FlywheelDebug/flwheelRamped?", isFlywheelAtThreshold());

    Logger.recordOutput("States/ShooterState", shooterState);
  }

  public void setShooterState(ShooterState state) {
    shooterState = state;
  }

  public Command startShooter() {
    return Commands.run(
        () -> {
          if (fixedShooter.getAsBoolean()) {
            setShooterState(ShooterState.SHOOT_FIXED);
          } else {
            if (allianceZoneTrigger.getAsBoolean()) setShooterState(ShooterState.SHOOT_HUB);
            if (leftPassTrigger.getAsBoolean()) setShooterState(ShooterState.SHOOT_PASS_L);
            if (rightPassTrigger.getAsBoolean()) setShooterState(ShooterState.SHOOT_PASS_R);
          }
        });
  }

  public Command startShooterOnce() {
    return Commands.runOnce(
        () -> {
          if (fixedShooter.getAsBoolean()) {
            setShooterState(ShooterState.SHOOT_FIXED);
          } else {
            if (allianceZoneTrigger.getAsBoolean()) setShooterState(ShooterState.SHOOT_HUB);
            if (leftPassTrigger.getAsBoolean()) setShooterState(ShooterState.SHOOT_PASS_L);
            if (rightPassTrigger.getAsBoolean()) setShooterState(ShooterState.SHOOT_PASS_R);
          }
        });
  }

  public Command idleShooter() {
    return Commands.runOnce(
        () -> {
          if (fixedShooter.getAsBoolean()) {
            setShooterState(ShooterState.IDLE_FIXED);
          } else {
            if (allianceZoneTrigger.getAsBoolean()) setShooterState(ShooterState.IDLE_HUB);
            if (leftPassTrigger.getAsBoolean()) setShooterState(ShooterState.IDLE_L);
            if (rightPassTrigger.getAsBoolean()) setShooterState(ShooterState.IDLE_R);
          }
        });
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

  public boolean isFlywheelAtThreshold() {
    return getFlywheelVelocities()[0].in(RPM) >= ShooterConstants.flywheelThreshold.in(RPM);
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

  @AutoLogOutput(key = "Shooter/Turret/MeasuredPositionRot")
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

  public AngularVelocity[] getFlywheelVelocities() {
    return new AngularVelocity[] {flywheelInputs.leftVelocity, flywheelInputs.rightVelocity};
  }

  public void setTurretSetpoint(Angle angle, AngularVelocity angvel) {
    turretHardware.setTurretSetpoint(angle, angvel);
  }

  public Pose2d getTurretFieldPose() {
    Pose2d turretBotPose =
        new Pose3d(poseSupplier.get()).transformBy(ShooterConstants.robotToTurret).toPose2d();
    Pose2d turretPoseOut =
        new Pose2d(
            turretBotPose.getX(),
            turretBotPose.getY(),
            turretBotPose
                .getRotation()
                .plus(Rotation2d.fromRotations(getTurretPosition()))
                .plus(Rotation2d.kCCW_Pi_2));
    return turretPoseOut;
  }

  @AutoLogOutput(key="Shooter/isWrapping")
  public boolean isWrapAround() {

    Angle firstLimit = turretSetPoint.minus(turretHardware.getTurretPosition());
    Angle secondLimit = turretHardware.getTurretPosition().minus(turretSetPoint);

    Angle preferred = Angle.ofBaseUnits(0, Rotations);
    if (Math.abs(firstLimit.baseUnitMagnitude()) > Math.abs(secondLimit.baseUnitMagnitude()))
    {
      preferred = secondLimit;
    }
    else
    {
      preferred = firstLimit;
    }

    if (Math.abs(preferred.in(Degrees)) > ShooterConstants.wrapAroundDegreesThreshold)
    {
      return true;
    }
    return false;
  }

  public BooleanSupplier wrapAroundSupplier() {
    return this::isWrapAround;
  }
}
