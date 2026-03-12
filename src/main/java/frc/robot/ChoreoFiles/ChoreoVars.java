package frc.robot.ChoreoFiles;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * Generated file containing variables defined in Choreo. DO NOT MODIFY THIS FILE YOURSELF; instead,
 * change these values in the Choreo GUI.
 */
public final class ChoreoVars {
  public static final LinearVelocity SpeedOverBump = Units.MetersPerSecond.of(2.25);
  public static final double WINDBACK_DIST = 2.6;
  public static final LinearVelocity SpeedThroughBalls = Units.MetersPerSecond.of(4);

  public static final class Poses {
    public static final Pose2d C_Climb = new Pose2d(1.7, 3.3, Rotation2d.fromRadians(3.142));
    public static final Pose2d C_Start = new Pose2d(3.575, 3.75, Rotation2d.kZero);
    public static final Pose2d CircleCenter = new Pose2d(4.625, 4.03, Rotation2d.kZero);
    public static final Pose2d DI_Start = new Pose2d(3.597, 5.6, Rotation2d.kZero);
    public static final Pose2d D_BumpAlliance =
        new Pose2d(3.2, 5.9, Rotation2d.fromRadians(-1.571));
    public static final Pose2d D_BumpNeutral = new Pose2d(6.1, 5.9, Rotation2d.fromRadians(-1.571));
    public static final Pose2d D_Climb = new Pose2d(1.61, 4.62, Rotation2d.kZero);
    public static final Pose2d D_Depot = new Pose2d(1.1, 5.97, Rotation2d.fromRadians(3.142));
    public static final Pose2d D_Intake = new Pose2d(7.8, 7, Rotation2d.fromRadians(-1.571));
    public static final Pose2d D_Intake_45 = new Pose2d(7.4, 6.5, Rotation2d.fromRadians(-0.785));
    public static final Pose2d D_Shoot = new Pose2d(3.1, 5.7, Rotation2d.fromRadians(-0.839));
    public static final Pose2d D_Start = new Pose2d(3.6, 5.9, Rotation2d.fromRadians(-1.571));
    public static final Pose2d H_BumpAlliance = new Pose2d(3.2, 2.5, Rotation2d.fromRadians(1.571));
    public static final Pose2d H_BumpNeutral = new Pose2d(6.1, 2.5, Rotation2d.fromRadians(1.571));
    public static final Pose2d H_Climb = new Pose2d(0.75, 2.625, Rotation2d.fromRadians(-1.571));
    public static final Pose2d H_HumanPlayer = new Pose2d(1.5, 0.65, Rotation2d.fromRadians(3.142));
    public static final Pose2d H_Intake = new Pose2d(7.8, 1.02, Rotation2d.fromRadians(1.571));
    public static final Pose2d H_Intake_45 = new Pose2d(7.4, 1.627, Rotation2d.fromRadians(0.785));
    public static final Pose2d H_Shoot = new Pose2d(3.1, 2.6, Rotation2d.kZero);
    public static final Pose2d H_ShootBack = new Pose2d(2.984, 1.6, Rotation2d.fromRadians(3.142));
    public static final Pose2d H_Start = new Pose2d(3.6, 2.5, Rotation2d.fromRadians(1.571));

    private Poses() {}
  }

  private ChoreoVars() {}
}
