package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotManager;
import frc.robot.RobotManager.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;

public class TeleopCommands {

  private Drive drive;
  private Intake intake;
  private Indexer indexer;
  private Transfer transfer;
  private Shooter shooter;
  private RobotManager manager;

  boolean fixedShooting = false;

  public TeleopCommands(
      Drive drive, Intake intake, Indexer indexer, Transfer transfer, Shooter shooter, RobotManager manager) {
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
    this.transfer = transfer;
    this.shooter = shooter;
    this.manager = manager;
  }

  public Command runIntake()
  {
    return manager.setIntakeCommand(IntakeManagerState.INTAKING);
  }

  public Command stopIntake()
  {
    return manager.setIntakeCommand(IntakeManagerState.IDLE);
  }

  public Command runShoot()
  {
    if (manager.robotState != RobotScoringState.CLIMBING) return manager.toStateCommand(correctShooterState());
    return Commands.none();
  }

  public Command stopShoot()
  {
    if (manager.robotState != RobotScoringState.CLIMBING)
    {
      return manager.toStateCommand(RobotScoringState.IDLE);
    }
    
    return Commands.none();
  }

  public Command toggleFixedShooting()
  {
    return Commands.runOnce(() -> {fixedShooting = !fixedShooting;});
  }

  public RobotScoringState correctShooterState()
  {
    return fixedShooting ? RobotScoringState.FIXED_SHOOTING : RobotScoringState.SHOOTING;
  }

}
