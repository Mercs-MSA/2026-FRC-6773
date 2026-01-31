package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

  private final IndexerIO kIndexerHardware;
  private final IndexerIOInputsAutoLogged kIndexerInputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO kIndexerIO) {
    this.kIndexerHardware = kIndexerIO;
  }

  @Override
  public void periodic() {
    kIndexerHardware.updateInputs(kIndexerInputs);
    Logger.processInputs("Indexer/Inputs", kIndexerInputs);
  }

  public void startIndexer(double velocity) {
    kIndexerHardware.setVelocity(velocity);
  }

  public void stopIndexer() {
    kIndexerHardware.stop();
  }
}
