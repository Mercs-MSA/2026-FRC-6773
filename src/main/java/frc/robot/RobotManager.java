package frc.robot;

import java.lang.reflect.Field;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.drive.Drive.Zone;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.Transfer.TransferState;
import frc.robot.util.geometry.AllianceFlipUtil;

public class RobotManager {

    private Drive drive;
    private Intake intake;
    private Indexer indexer;
    private Transfer transfer;
    private Shooter shooter;

    private RobotScoringState robotState;
    private ShooterState shooterState;

    public RobotManager(
        Drive drive,
        Intake intake,
        Indexer indexer,
        Transfer transfer,
        Shooter shooter
    ) {
        this.drive = drive;
        this.intake = intake;
        this.indexer = indexer;
        this.transfer = transfer;
        this.shooter = shooter;
        robotState = RobotScoringState.IDLE;
    }

    public void periodicManager() {
        // MARK: - INTAKE
        switch (robotState)
        {
            case CLIMBING:
            case IDLE:
                intake.setIntakeState(IntakeState.IDLE);
                break;
            case INTAKING:
            case SHOOTING_INTAKING:
                intake.setIntakeState(IntakeState.INTAKING);
                break;
            case SHOOTING:
                intake.setIntakeState(IntakeState.AGITATE);
                break;
            default:
                break;
        }

        // MARK: - SPINDEXER
        switch (robotState) {
            case CLIMBING:
            case INTAKING:
            case IDLE:
                indexer.setIndexerState(IndexerState.IDLE);
                break;
            case FIXED_SHOOTING:
            case SHOOTING:
            case SHOOTING_INTAKING:
                indexer.setIndexerState(IndexerState.INDEXING);
                break;
            default:
                break;   
        }

        // MARK: - TRANSFER
        switch (robotState) {
            case CLIMBING:
            case INTAKING:
            case IDLE:
                transfer.setTransferState(TransferState.IDLE);
                break;
            case FIXED_SHOOTING:
            case SHOOTING:
            case SHOOTING_INTAKING:
                transfer.setTransferState(TransferState.TRANSFERRING);
                break;
            default:
                break;
        }

        // MARK: - SHOOTER
        switch (robotState) {
            case CLIMBING:
                shooterState = ShooterState.IDLE;
                break;
            case INTAKING:
            case IDLE:
                shooterState = ShooterState.IDLE_HUB;
                break;
            case FIXED_SHOOTING:
                shooterState = ShooterState.SHOOT_FIXED;
                break;
            // The following two states need more complex logic
            case SHOOTING:
            case SHOOTING_INTAKING:
                Zone zone = drive.returnZone(drive.getPose());
                switch (zone) {
                    case ALLIANCE:
                        shooterState = ShooterState.SHOOT_HUB;
                        break;
                    case NEUTRAL_LEFT:
                        shooterState = ShooterState.SHOOT_PASS_L;
                        break;
                    case NEUTRAL_RIGHT:
                        shooterState = ShooterState.SHOOT_PASS_R;
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;
        }
        shooter.setShooterState(shooterState);

        //MARK: - DRIVE
        switch(robotState)
        {
            case CLIMBING:
                drive.setDriveState(DriveState.ALIGN);
            case IDLE:
            case INTAKING:
                drive.setDriveState(DriveState.DRIVING);
                break;
            case SHOOTING:
            case SHOOTING_INTAKING:
                drive.setDriveState(DriveState.SHOOTING);
                break;
            case FIXED_SHOOTING:
            default:
                drive.setDriveState(DriveState.IDLE);
                break;
        }

        //MARK: - CLIMB(TODO)
        //ADD CLIMBER HERE WHEN DONE
    }

    public enum RobotScoringState {
        IDLE,
        INTAKING,
        SHOOTING,
        CLIMBING,
        FIXED_SHOOTING,
        SHOOTING_INTAKING,
    }

    public enum ShootGoal {
        HUB(AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint)),
        LEFT_PASS(AllianceFlipUtil.apply(FieldConstants.LeftBump.farLeftCorner)),
        RIGHT_PASS(AllianceFlipUtil.apply(FieldConstants.RightBump.farRightCorner));

        private Translation3d goal;

        private ShootGoal(Translation3d goal) {
            this.goal = goal;
        }

        private ShootGoal(Translation2d goal) {
            this.goal = new Translation3d(goal);
        }

        public Translation3d getGoal() {
            return goal;
        }
    }
}
