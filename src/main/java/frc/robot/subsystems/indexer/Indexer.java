package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  public void setVelocity(double velocity) {
    kIndexerHardware.setVelocity(velocity);
  }

  @AutoLogOutput(key="Indexer/Outputs/Velocity")
  public double getVelocity()
  {
    return kIndexerHardware.getVelocity();
  }

  public void stopIndexer() {
    kIndexerHardware.stop();
  }
}
