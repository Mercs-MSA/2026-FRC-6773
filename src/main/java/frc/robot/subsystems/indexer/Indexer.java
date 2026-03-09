package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.InchesPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  public enum IndexerState {
    IDLE,
    INDEXING
  }

  public IndexerState indexerState;

  private final IndexerSpindexerIO spindexerHardware;
  private final IndexerSpindexerIOInputsAutoLogged spindexerInputs =
      new IndexerSpindexerIOInputsAutoLogged();

  private final IndexerKickerIO kickerHardware;
  private final IndexerKickerIOInputsAutoLogged kickerInputs =
      new IndexerKickerIOInputsAutoLogged();

  public Indexer(IndexerSpindexerIO spindexerIO, IndexerKickerIO kickerIO) {
    spindexerHardware = spindexerIO;
    kickerHardware = kickerIO;
    indexerState = IndexerState.IDLE;
  }

  @Override
  public void periodic() {
    spindexerHardware.updateInputs(spindexerInputs);
    Logger.processInputs("Indexer/Inputs/Spindexer", spindexerInputs);
    kickerHardware.updateInputs(kickerInputs);
    Logger.processInputs("Indexer/Inputs/Kicker", kickerInputs);

    Logger.recordOutput(
        "Indexer/SpindexerVelocityIPS", spindexerInputs.linearVelocity.in(InchesPerSecond));
    Logger.recordOutput(
        "Indexer/KickerVelocityIPS", kickerInputs.linearVelocity.in(InchesPerSecond));

    switch (indexerState) {
      case IDLE:
        stopKicker();
        stopSpindexer();
        break;
      case INDEXING:
        setSpindexerTangentialVelocity(Constants.fuelLaunchVelocity);
        setKickerTangentialVelocity(Constants.fuelLaunchVelocity);
        break;
    }
  }

  public void setIndexerState(IndexerState state) {
    indexerState = state;
  }

  public void setSpindexerVoltage(double voltage) {
    spindexerHardware.setVoltage(voltage);
  }

  public void setKickerVoltage(double voltage) {
    kickerHardware.setVoltage(voltage);
  }

  public void setSpindexerAngularVelocity(AngularVelocity velocity) {
    spindexerHardware.setAngularVelocity(velocity);
  }

  public void setKickerAngularVelocity(AngularVelocity velocity) {
    kickerHardware.setAngularVelocity(velocity);
  }

  public void setSpindexerTangentialVelocity(LinearVelocity velocity) {
    spindexerHardware.setTangentialVelocity(velocity);
  }

  public void setKickerTangentialVelocity(LinearVelocity velocity) {
    kickerHardware.setTangentialVelocity(velocity);
  }

  public void stopSpindexer() {
    spindexerHardware.stop();
  }

  public void stopKicker() {
    kickerHardware.stop();
  }

  @AutoLogOutput(key = "States/IndexerState")
  public IndexerState getIndexerState() {
    return indexerState;
  }
}
