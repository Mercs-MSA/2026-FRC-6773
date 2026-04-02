package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import java.util.function.Supplier;

public class ZoneUtil {
  private static final PredictiveXBaseZone BLUE_BOTTOM_BUMP =
      new PredictiveXBaseZone(
          FieldConstants.RightBump.nearRightCorner.getMeasureX(),
          FieldConstants.RightBump.farRightCorner.getMeasureX(),
          FieldConstants.RightBump.nearRightCorner.getMeasureY(),
          FieldConstants.RightBump.nearLeftCorner.getMeasureY());
  private static final PredictiveXBaseZone BLUE_TOP_BUMP = BLUE_BOTTOM_BUMP.mirroredY();
  private static final PredictiveXBaseZone RED_BOTTOM_BUMP = BLUE_BOTTOM_BUMP.mirroredX();
  private static final PredictiveXBaseZone RED_TOP_BUMP = BLUE_TOP_BUMP.mirroredX();

  public static final PredictiveXZoneCollection BUMP_ZONES =
      new PredictiveXZoneCollection(BLUE_BOTTOM_BUMP, BLUE_TOP_BUMP, RED_BOTTOM_BUMP, RED_TOP_BUMP);

  public static final BaseZone BLUE_RIGHT_PASS_ZONE =
      new BaseZone(
          FieldConstants.LinesVertical.neutralZoneNear,
          FieldConstants.fieldLength,
          0.0,
          FieldConstants.LinesHorizontal.center);
  public static final BaseZone BLUE_LEFT_PASS_ZONE = BLUE_RIGHT_PASS_ZONE.mirroredY();

  public static final BaseZone BLUE_ALLIANCE_ZONE =
      new BaseZone(-2, FieldConstants.LinesVertical.allianceZone, -2.0, FieldConstants.fieldWidth);

  public static final BaseZone BLUE_ALLIANCE_TRENCH_ZONE_RIGHT =
      new BaseZone(
          FieldConstants.LinesVertical.allianceZone,
          FieldConstants.LinesVertical.hubCenter,
          0,
          FieldConstants.LinesHorizontal.rightTrenchOpenStart);

  public static final BaseZone BLUE_ALLIANCE_TRENCH_ZONE_LEFT =
      new BaseZone(
          FieldConstants.LinesVertical.allianceZone,
          FieldConstants.LinesVertical.hubCenter,
          FieldConstants.LinesHorizontal.leftTrenchOpenEnd,
          FieldConstants.fieldWidth);

  public static final BaseZone RED_LEFT_PASS_ZONE = BLUE_RIGHT_PASS_ZONE.mirroredX();

  public static final BaseZone RED_RIGHT_PASS_ZONE = BLUE_LEFT_PASS_ZONE.mirroredX();

  public static final BaseZone RED_ALLIANCE_ZONE = BLUE_ALLIANCE_ZONE.mirroredX();

  public static final BaseZone RED_ALLIANCE_TRENCH_ZONE_RIGHT =
      BLUE_ALLIANCE_TRENCH_ZONE_LEFT.mirroredX();

  public static final BaseZone RED_ALLIANCE_TRENCH_ZONE_LEFT =
      BLUE_ALLIANCE_TRENCH_ZONE_RIGHT.mirroredX();

  // public static final BaseZone

  public static final ZoneCollection BLUE_ALLIANCE_ZONES =
      new ZoneCollection(
          BLUE_ALLIANCE_ZONE, BLUE_ALLIANCE_TRENCH_ZONE_RIGHT, BLUE_ALLIANCE_TRENCH_ZONE_LEFT);

  public static final ZoneCollection RED_ALLIANCE_ZONES =
      new ZoneCollection(
          RED_ALLIANCE_ZONE, RED_ALLIANCE_TRENCH_ZONE_RIGHT, RED_ALLIANCE_TRENCH_ZONE_LEFT);

  public static interface Zone {
    public Trigger contains(Supplier<Pose2d> pose);
  }

  public static interface PredictiveXZone extends Zone {
    public Trigger willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt);
  }

  public static class BaseZone implements Zone {
    protected final double xMin, xMax, yMin, yMax;

    public BaseZone(double xMin, double xMax, double yMin, double yMax) {
      this.xMin = xMin;
      this.xMax = xMax;
      this.yMin = yMin;
      this.yMax = yMax;
    }

    public BaseZone(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
      this(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters));
    }

    @Override
    public Trigger contains(Supplier<Pose2d> poseSupplier) {
      return new Trigger(() -> this.containsPoint(poseSupplier.get().getTranslation()));
    }

    protected boolean containsPoint(Translation2d point) {
      return point.getX() >= xMin
          && point.getX() <= xMax
          && point.getY() >= yMin
          && point.getY() <= yMax;
    }

    public BaseZone mirroredX() {
      return new BaseZone(
          FieldConstants.fieldLength - xMax, FieldConstants.fieldLength - xMin, yMin, yMax);
    }

    public BaseZone mirroredY() {
      return new BaseZone(
          xMin, xMax, FieldConstants.fieldWidth - yMax, FieldConstants.fieldWidth - yMin);
    }

    /** list of corners, with the bottom left corner repeated at the end to form a closed loop */
    public Translation2d[] getCorners() {
      return new Translation2d[] {
        new Translation2d(xMin, yMin),
        new Translation2d(xMax, yMin),
        new Translation2d(xMax, yMax),
        new Translation2d(xMin, yMax),
        new Translation2d(xMin, yMin)
      };
    }
  }

  public static class PredictiveXBaseZone extends BaseZone implements PredictiveXZone {
    public PredictiveXBaseZone(double xMin, double xMax, double yMin, double yMax) {
      super(xMin, xMax, yMin, yMax);
    }

    public PredictiveXBaseZone(BaseZone baseZone) {
      super(baseZone.xMin, baseZone.xMax, baseZone.yMin, baseZone.yMax);
    }

    public PredictiveXBaseZone(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
      super(xMin, xMax, yMin, yMax);
    }

    @Override
    public Trigger willContain(
        Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt) {
      return new Trigger(
          () -> willContainPoint(pose.get().getTranslation(), fieldSpeeds.get(), dt));
    }

    protected boolean willContainPoint(Translation2d point, ChassisSpeeds fieldSpeeds, Time dt) {
      return (point.getY() >= yMin && point.getY() <= yMax)
          && ((point.getX() >= xMin && point.getX() <= xMax)
              || (point.getX() < xMin
                  && fieldSpeeds.vxMetersPerSecond * dt.in(Seconds) >= xMin - point.getX())
              || (point.getX() > xMax
                  && fieldSpeeds.vxMetersPerSecond * dt.in(Seconds) <= xMax - point.getX()));
    }

    @Override
    public PredictiveXBaseZone mirroredX() {
      return new PredictiveXBaseZone(super.mirroredX());
    }

    @Override
    public PredictiveXBaseZone mirroredY() {
      return new PredictiveXBaseZone(super.mirroredY());
    }
  }

  public static class ZoneCollection implements Zone {
    protected final Zone[] zones;

    public ZoneCollection(Zone... zones) {
      this.zones = zones;
    }

    @Override
    public Trigger contains(Supplier<Pose2d> pose) {
      Trigger combined = new Trigger(() -> false);

      for (Zone zone : zones) {
        combined = combined.or(zone.contains(pose));
      }

      return combined;
    }
  }

  public static class PredictiveXZoneCollection extends ZoneCollection implements PredictiveXZone {
    public PredictiveXZoneCollection(PredictiveXZone... zones) {
      super(zones);
    }

    @Override
    public Trigger willContain(
        Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt) {
      Trigger combined = new Trigger(() -> false);

      for (Zone zone : zones) {
        combined = combined.or(((PredictiveXZone) zone).willContain(pose, fieldSpeeds, dt));
      }

      return combined;
    }
  }
}
