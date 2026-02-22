package frc.robot.ChoreoFiles;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * Generated file containing variables defined in Choreo.
 * DO NOT MODIFY THIS FILE YOURSELF; instead, change these values
 * in the Choreo GUI.
 */
public final class ChoreoVars {
    public static final LinearVelocity SpeedOverBump = Units.MetersPerSecond.of(2);
    public static final double WINDBACK_DIST = 2.6;

    public static final class Poses {
        public static final Pose2d C_Climb = new Pose2d(1.7, 3.3, Rotation2d.fromRadians(3.142));
        public static final Pose2d C_Start = new Pose2d(3.575, 3.75, Rotation2d.kZero);
        public static final Pose2d CircleCenter = new Pose2d(4.625, 4.03, Rotation2d.kZero);
        public static final Pose2d DI_Start = new Pose2d(3.6, 5.6, Rotation2d.fromRadians(-1.571));
        public static final Pose2d D_Climb = new Pose2d(1.61, 4.62, Rotation2d.kZero);
        public static final Pose2d D_Depot = new Pose2d(1.1, 6, Rotation2d.fromRadians(3.142));
        public static final Pose2d D_Intake = new Pose2d(7.8, 7, Rotation2d.fromRadians(-1.571));
        public static final Pose2d D_Shoot = new Pose2d(3.1, 5.7, Rotation2d.fromRadians(-0.839));
        public static final Pose2d HI_Start = new Pose2d(3.6, 2.15, Rotation2d.fromRadians(1.571));
        public static final Pose2d H_Climb = new Pose2d(0.75, 2.625, Rotation2d.fromRadians(-1.571));
        public static final Pose2d H_HumanPlayer = new Pose2d(0.8, 0.65, Rotation2d.fromRadians(3.142));
        public static final Pose2d H_Intake = new Pose2d(7.8, 1.02, Rotation2d.fromRadians(1.571));
        public static final Pose2d H_Shoot = new Pose2d(3.1, 2.6, Rotation2d.fromRadians(0.757));
        public static final Pose2d D_Intake_45 = new Pose2d(7.215, 5.6, Rotation2d.fromRadians(-0.785));

        private Poses() {}
    }

    private ChoreoVars() {}
}