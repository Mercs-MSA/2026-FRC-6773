package frc.robot.commands;

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

  public TeleopCommands(
      Drive drive, Intake intake, Indexer indexer, Transfer transfer, Shooter shooter) {
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
    this.transfer = transfer;
    this.shooter = shooter;
  }
}
