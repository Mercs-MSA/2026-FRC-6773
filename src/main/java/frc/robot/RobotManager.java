// package frc.robot;

// import java.lang.reflect.Field;

// import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import frc.robot.constants.FieldConstants;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.indexer.Indexer;
// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.subsystems.transfer.Transfer;
// import frc.robot.util.geometry.AllianceFlipUtil;

// public class RobotManager {

//     private Drive drive;
//     private Intake intake;
//     private Indexer indexer;
//     private Transfer transfer;
//     private Shooter shooter;

//     private RobotScoringState robotState;

//     public RobotManager(
//         Drive drive,
//         Intake intake,
//         Indexer indexer,
//         Transfer transfer,
//         Shooter shooter
//     ) {
//         this.drive = drive;
//         this.intake = intake;
//         this.indexer = indexer;
//         this.transfer = transfer;
//         this.shooter = shooter;
//         robotState = RobotScoringState.IDLE;
//     }

//     public void periodicManager() {
//         switch (robotState) {
//             case IDLE:

//         }
//     }

//     public enum RobotScoringState {
//         IDLE,
//         INTAKING,
//         SHOOTING,
//         CLIMBING,
//         FIXED_SHOOTING
//     }

//     public enum ShootGoal {
//         HUB(AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint)),
//         LEFT_PASS(AllianceFlipUtil.apply(FieldConstants.LeftBump.farLeftCorner)),
//         RIGHT_PASS(AllianceFlipUtil.apply(FieldConstants.RightBump.farRightCorner));

//         private Translation3d goal;

//         private ShootGoal(Translation3d goal) {
//             this.goal = goal;
//         }

//         private ShootGoal(Translation2d goal) {
//             this.goal = new Translation3d(goal);
//         }

//         public Translation3d getGoal() {
//             return goal;
//         }
//     }
// }
