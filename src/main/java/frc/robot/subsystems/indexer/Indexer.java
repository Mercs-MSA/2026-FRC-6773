package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import java.util.function.BooleanSupplier;

public class Indexer extends SubsystemBase {
  public enum IndexerState {
    IDLE,
    INDEXING,
    JAM,
    FLUSH
  }

  public IndexerState indexerState;
  public BooleanSupplier wrapSupplier;

  private final Timer jammedTimer = new Timer();
  private final Timer unJamTimer = new Timer();
  private boolean jammed = false;

  private final IndexerSpindexerIO spindexerHardware;
  private final IndexerSpindexerIOInputsAutoLogged spindexerInputs =
      new IndexerSpindexerIOInputsAutoLogged();

  private final IndexerKickerIO kickerHardware;
  private final IndexerKickerIOInputsAutoLogged kickerInputs =
      new IndexerKickerIOInputsAutoLogged();

  public Indexer(
      IndexerSpindexerIO spindexerIO,
      IndexerKickerIO kickerIO,
      BooleanSupplier wrapBooleanSupplier) {
    spindexerHardware = spindexerIO;
    kickerHardware = kickerIO;
    indexerState = IndexerState.IDLE;
    wrapSupplier = wrapBooleanSupplier;
  }

  @Override
  public void periodic() {
    spindexerHardware.updateInputs(spindexerInputs);
    // Logger.processInputs("Indexer/Inputs/Spindexer", spindexerInputs);
    kickerHardware.updateInputs(kickerInputs);
    // Logger.processInputs("Indexer/Inputs/Kicker", kickerInputs);

    // Logger.recordOutput(
    //     "Indexer/SpindexerVelocityIPS", spindexerInputs.linearVelocity.in(InchesPerSecond));
    // Logger.recordOutput(
    //     "Indexer/KickerVelocityIPS", kickerInputs.linearVelocity.in(InchesPerSecond));

    // Check for jams first (can override state)
    if (indexerState == IndexerState.INDEXING) {
      jamCheck(RotationsPerSecond.of(0.5));
    }

    // State machine
    switch (indexerState) {
      case IDLE:
        stopKicker();
        stopSpindexer();
        break;
      case INDEXING:
        if (wrapSupplier.getAsBoolean()) {
          stopKicker();
          stopSpindexer();
        } else {
          setSpindexerAngularVelocity(RotationsPerSecond.of(150));
          setKickerTangentialVelocity(Constants.fuelLaunchVelocity);
        }
        break;
      case JAM:
        setKickerTangentialVelocity(Constants.fuelLaunchVelocity.times(-0.5));
        setSpindexerAngularVelocity(RotationsPerSecond.of(-33));
        break;
      case FLUSH:
        setKickerTangentialVelocity(Constants.fuelLaunchVelocity);
        stopSpindexer();
        break;
    }
  }

  public void jamCheck(AngularVelocity threshold) {
    if (jammedTimer.isRunning() && jammedTimer.hasElapsed(Seconds.of(0.4))) {
      jammed = true;
      unJamTimer.start();
      jammedTimer.stop();
      jammedTimer.reset();
      setIndexerState(IndexerState.JAM);
    }
    if (jammed && unJamTimer.isRunning() && unJamTimer.hasElapsed(0.1)) {
      jammed = false;
      unJamTimer.stop();
      unJamTimer.reset();
      setIndexerState(IndexerState.INDEXING);
    }
    if ((getKickerVelocity().abs(RotationsPerSecond) < threshold.in(RotationsPerSecond)
            || getSpindexerVelocity().abs(RotationsPerSecond) < threshold.in(RotationsPerSecond))
        && !jammed) {
      jammedTimer.start();
    } else if (jammedTimer.isRunning()) {
      jammedTimer.stop();
      jammedTimer.reset();
    }
  }

  public void setIndexerState(IndexerState state) {
    indexerState = state;
  }

  public void setSpindexerVoltage(double voltage) {
    spindexerHardware.setVoltage(voltage);
  }

  public AngularVelocity getKickerVelocity() {
    return kickerInputs.angularVelocity;
  }

  public AngularVelocity getSpindexerVelocity() {
    return spindexerInputs.angularVelocity;
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

  // @AutoLogOutput(key = "States/IndexerState")
  public IndexerState getIndexerState() {
    return indexerState;
  }
}
