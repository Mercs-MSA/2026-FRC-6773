// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.FieldConstants;
import frc.robot.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ShooterTurretCalculator {

  public static final InterpolatingTreeMap<Double, ShotData> SHOT_MAP =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotData::interpolate);
  public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();

  public static Pose2d lastLookAhead = new Pose2d();

  static {
    SHOT_MAP.put(7.2, new ShotData(RPM.of(90 * 60), Degrees.of(20)));
    TOF_MAP.put(7.2, 1.46);

    SHOT_MAP.put(6.7, new ShotData(RPM.of(83 * 60), Degrees.of(20)));
    TOF_MAP.put(6.7, 1.46);

    SHOT_MAP.put(6.3, new ShotData(RPM.of(80 * 60), Degrees.of(20)));
    TOF_MAP.put(6.3, 1.46);

    SHOT_MAP.put(5.9, new ShotData(RPM.of(77 * 60), Degrees.of(20)));
    TOF_MAP.put(5.9, 1.46);

    SHOT_MAP.put(5.6, new ShotData(RPM.of(74 * 60), Degrees.of(20)));
    TOF_MAP.put(5.6, 1.46);

    SHOT_MAP.put(5.5, new ShotData(RPM.of(74 * 60), Degrees.of(20)));
    TOF_MAP.put(5.5, 1.46);

    SHOT_MAP.put(5.18, new ShotData(RPM.of(70 * 60), Degrees.of(19)));
    TOF_MAP.put(5.18, 1.37);

    SHOT_MAP.put(4.55, new ShotData(RPM.of(65 * 60), Degrees.of(18)));
    TOF_MAP.put(4.55, 1.25);

    SHOT_MAP.put(4.082, new ShotData(RPM.of(64 * 60), Degrees.of(16)));
    TOF_MAP.put(4.082, 1.26);

    SHOT_MAP.put(3.483, new ShotData(RPM.of(60 * 60), Degrees.of(12)));
    TOF_MAP.put(3.483, 1.24);

    SHOT_MAP.put(3.022, new ShotData(RPM.of(57 * 60), Degrees.of(12)));
    TOF_MAP.put(3.022, 1.22);

    SHOT_MAP.put(2.7, new ShotData(RPM.of(55 * 60), Degrees.of(10)));
    TOF_MAP.put(2.7, 1.18);

    SHOT_MAP.put(2.58, new ShotData(RPM.of(52 * 60), Degrees.of(8)));
    TOF_MAP.put(2.58, 1.15);

    SHOT_MAP.put(2.012, new ShotData(RPM.of(52 * 60), Degrees.of(5)));
    TOF_MAP.put(2.012, 1.09);

    SHOT_MAP.put(1.253, new ShotData(RPM.of(50 * 60), Degrees.of(1)));
    TOF_MAP.put(1.253, 1.01);
  }

  public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
    return Meters.of(robot.getTranslation().getDistance(target.toTranslation2d()));
  }

  // see https://www.desmos.com/geometry/l4edywkmha
  public static Angle calculateAngleFromVelocity(
      Pose2d robot, LinearVelocity velocity, Translation3d target) {
    double g = MetersPerSecondPerSecond.of(9.81).in(InchesPerSecondPerSecond);
    double vel = velocity.in(InchesPerSecond);
    double x_dist = getDistanceToTarget(robot, target).in(Inches);
    double y_dist = target.getMeasureZ().minus(robotToTurret.getMeasureZ()).in(Inches);
    double angle =
        Math.atan(
            ((vel * vel)
                    + Math.sqrt(
                        Math.pow(vel, 4) - g * (g * x_dist * x_dist + 2 * y_dist * vel * vel)))
                / (g * x_dist));
    return Radians.of(angle);
  }

  // calculates how long it will take for a projectile to travel a set distance given its initial
  // velocity and angle
  public static Time calculateTimeOfFlight(
      LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
    double vel = exitVelocity.in(MetersPerSecond);
    double angle = Math.PI / 2 - hoodAngle.in(Radians);
    double dist = distance.in(Meters);
    return Seconds.of(dist / (vel * Math.cos(angle)));
  }

  public static AngularVelocity linearToAngularVelocity(LinearVelocity vel, Distance radius) {
    return RadiansPerSecond.of(vel.in(MetersPerSecond) / radius.in(Meters));
  }

  public static LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
    return MetersPerSecond.of(vel.in(RadiansPerSecond) * radius.in(Meters));
  }

  // calculates the angle of a turret relative to the robot to hit a target
  public static Angle calculateAzimuthAngle(
      Pose2d robot, Translation3d target, Angle currentAngle) {
    Translation2d turretTranslation =
        new Pose3d(robot).transformBy(ShooterConstants.robotToTurret).toPose2d().getTranslation();

    Translation2d direction = target.toTranslation2d().minus(turretTranslation);
    return calculateAzimuthAngle(robot, direction.getAngle().getMeasure(), currentAngle);
  }

  // calculates the angle of a turret relative to the robot to hit a target
  public static Angle calculateAzimuthAngle(
      Pose2d robot, Angle fieldRelativeAngle, Angle currentAngle) {
    double angle =
        MathUtil.inputModulus(
            new Rotation2d(fieldRelativeAngle)
                .minus(robot.getRotation())
                .minus(Rotation2d.fromDegrees(90))
                .getRotations(),
            -0.5,
            0.5);
    double current = currentAngle.in(Rotations);
    if (current > 0 && angle + 1 <= ShooterConstants.turretMaxLimit.getRotations()) angle += 1;
    if (current < 0 && angle - 1 >= ShooterConstants.turretMinLimit.getRotations()) angle -= 1;

    angle = MathUtil.clamp(angle, turretMinLimit.getRotations(), turretMaxLimit.getRotations());

    Logger.recordOutput("Turret/DesiredAzimuthRad", angle);
    return Rotations.of(angle);
  }

  // Calculate the velocity of the turret accounting for robot rotation
  public static ChassisSpeeds getTurretVelocity(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {
    // Get turret position relative to robot center
    Pose2d turretPose = (new Pose3d(robotPose).transformBy(robotToTurret)).toPose2d();

    // Offset from robot center to turret
    Translation2d offset = turretPose.getTranslation().minus(robotPose.getTranslation());

    // When robot rotates, turret gets additional velocity: omega × offset
    // In 2D: vx_additional = -omega * offset_y, vy_additional = omega * offset_x
    double omega = fieldSpeeds.omegaRadiansPerSecond;
    double vxAdditional = -omega * offset.getY();
    double vyAdditional = omega * offset.getX();

    // Total turret velocity
    return new ChassisSpeeds(
        fieldSpeeds.vxMetersPerSecond + vxAdditional,
        fieldSpeeds.vyMetersPerSecond + vyAdditional,
        omega);
  }

  // Move a target a set time in the future accounting for turret velocity
  public static Translation3d predictTargetPos(
      Translation3d target, Pose2d robotPose, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
    ChassisSpeeds turretVelocity = getTurretVelocity(robotPose, fieldSpeeds);

    double predictedX = target.getX() - turretVelocity.vxMetersPerSecond * timeOfFlight.in(Seconds);
    double predictedY = target.getY() - turretVelocity.vyMetersPerSecond * timeOfFlight.in(Seconds);

    Logger.recordOutput(
        "Turret/lookAheadPose", new Pose2d(predictedX, predictedY, Rotation2d.kZero));

    return new Translation3d(predictedX, predictedY, target.getZ());
  }

  // Calculate turret-centric azimuth velocity (accounts for rotation + linear motion effects)
  public static AngularVelocity getTurretAzimuthVelocity(
      Pose2d robot, Translation3d target, ChassisSpeeds fieldSpeeds) {
    // Base counter-rotation to maintain aim during robot spin
    double baseCounterRotation = -fieldSpeeds.omegaRadiansPerSecond;

    // Additional compensation for linear motion changing azimuth
    Pose2d turretPose = (new Pose3d(robot).transformBy(robotToTurret)).toPose2d();
    Translation2d toTarget = target.toTranslation2d().minus(turretPose.getTranslation());
    double distToTarget = toTarget.getNorm();

    // Rate of azimuth change due to linear motion: dθ/dt = (v_perp) / distance
    // v_perp is the component of velocity perpendicular to the target direction
    double vPerp =
        -fieldSpeeds.vxMetersPerSecond * Math.sin(toTarget.getAngle().getRadians())
            + fieldSpeeds.vyMetersPerSecond * Math.cos(toTarget.getAngle().getRadians());
    double azimuthChangeFromLinear = vPerp / Math.max(distToTarget, 0.1);

    return RadiansPerSecond.of(baseCounterRotation + azimuthChangeFromLinear);
  }

  // Move a target a set time in the future along a velocity defined by fieldSpeeds
  public static Translation3d predictTargetPos(
      Translation3d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
    double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight.in(Seconds);
    double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight.in(Seconds);

    Logger.recordOutput(
        "Turret/lookAheadPose", new Pose2d(predictedX, predictedY, Rotation2d.kZero));

    return new Translation3d(predictedX, predictedY, target.getZ());
  }

  // see https://www.desmos.com/calculator/ezjqolho6g
  public static ShotData calculateShotFromFunnelClearance(
      Pose2d robot, Translation3d actualTarget, Translation3d predictedTarget) {
    Pose2d turretPose = (new Pose3d(robot).transformBy(robotToTurret)).toPose2d();
    double x_dist = getDistanceToTarget(turretPose, predictedTarget).in(Inches);
    double y_dist = predictedTarget.getMeasureZ().minus(robotToTurret.getMeasureZ()).in(Inches);
    double g = 386;
    double r =
        Units.metersToInches(FieldConstants.Hub.width)
            * x_dist
            / getDistanceToTarget(robot, actualTarget).in(Inches);
    double h = Units.metersToInches(FieldConstants.Hub.height);
    double A1 = x_dist * x_dist;
    double B1 = x_dist;
    double D1 = y_dist;
    double A2 = -x_dist * x_dist + (x_dist - r) * (x_dist - r);
    double B2 = -r;
    double D2 = h;
    double Bm = -B2 / B1;
    double A3 = Bm * A1 + A2;
    double D3 = Bm * D1 + D2;
    double a = D3 / A3;
    double b = (D1 - A1 * a) / B1;
    double theta = Math.atan(b);
    double v0 = Math.sqrt(-g / (2 * a * (Math.cos(theta)) * (Math.cos(theta))));
    if (Double.isNaN(v0) || Double.isNaN(theta)) {
      v0 = 0;
      theta = 0;
    }
    return new ShotData(
        linearToAngularVelocity(InchesPerSecond.of(v0), Distance.ofBaseUnits(2, Inches)),
        Radians.of(Math.PI / 2 - theta),
        predictedTarget);
  }

  // use an iterative lookahead approach to determine shot parameters for a moving robot
  public static ShotData iterativeMovingShotFromFunnelClearance(
      Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {
    // Perform initial estimation (assuming unmoving robot) to get time of flight estimate
    ShotData shot = calculateShotFromFunnelClearance(robot, target, target);
    Distance distance = getDistanceToTarget(robot, target);
    Time timeOfFlight =
        calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(), distance);
    Translation3d predictedTarget = target;

    // Iterate the process, getting better time of flight estimations and updating the predicted
    // target accordingly
    for (int i = 0; i < iterations; i++) {
      predictedTarget = predictTargetPos(target, robot, fieldSpeeds, timeOfFlight);
      shot = calculateShotFromFunnelClearance(robot, target, predictedTarget);
      timeOfFlight =
          calculateTimeOfFlight(
              shot.getExitVelocity(),
              shot.getHoodAngle(),
              getDistanceToTarget(robot, predictedTarget));
    }

    return shot;
  }

  public static ShotData iterativeMovingShotFromMap(
      Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {
    target = AllianceFlipUtil.apply(target);
    Pose2d turretPose = (new Pose3d(robot).transformBy(robotToTurret)).toPose2d();
    double distance = getDistanceToTarget(turretPose, target).in(Meters);
    ShotData shot = SHOT_MAP.get(distance);
    shot = new ShotData(shot.exitVelocity, shot.hoodAngle, target);
    Time timeOfFlight = Seconds.of(TOF_MAP.get(distance));
    Translation3d predictedTarget = target;

    // Iterate the process, getting better time of flight estimations and updating the predicted
    // target accordingly
    for (int i = 0; i < iterations; i++) {
      predictedTarget = predictTargetPos(target, robot, fieldSpeeds, timeOfFlight);
      lastLookAhead = new Pose2d(predictedTarget.getX(), predictedTarget.getY(), Rotation2d.kZero);
      distance = getDistanceToTarget(robot, predictedTarget).in(Meters);
      shot = SHOT_MAP.get(distance);
      shot = new ShotData(shot.exitVelocity, shot.hoodAngle, predictedTarget);
      timeOfFlight = Seconds.of(TOF_MAP.get(distance));
    }

    return shot;
  }

  public record ShotData(double exitVelocity, double hoodAngle, Translation3d target) {
    public ShotData(AngularVelocity exitVelocity, Angle hoodAngle, Translation3d target) {
      this(exitVelocity.in(RadiansPerSecond), hoodAngle.in(Radians), target);
    }

    public ShotData(AngularVelocity exitVelocity, Angle hoodAngle) {
      this(exitVelocity, hoodAngle, FieldConstants.Hub.topCenterPoint);
    }

    public ShotData(double exitVelocity, double hoodAngle) {
      this(exitVelocity, hoodAngle, FieldConstants.Hub.topCenterPoint);
    }

    public LinearVelocity getExitVelocity() {
      return angularToLinearVelocity(
          RadiansPerSecond.of(this.exitVelocity), Distance.ofBaseUnits(2, Inches));
    }

    public Angle getHoodAngle() {
      return Radians.of(this.hoodAngle);
    }

    public Translation3d getTarget() {
      return this.target;
    }

    public static ShotData interpolate(ShotData start, ShotData end, double t) {
      return new ShotData(
          MathUtil.interpolate(start.exitVelocity, end.exitVelocity, t),
          MathUtil.interpolate(start.hoodAngle, end.hoodAngle, t),
          end.target);
    }
  }
}
