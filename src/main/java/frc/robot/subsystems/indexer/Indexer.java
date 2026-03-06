package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.transfer.TransferConstants.TransferHardware;

public class Indexer extends SubsystemBase {
    public enum IndexerState {
        IDLE,
        INDEXING
    }

    public IndexerState indexerState;

    private final IndexerSpindexerIO spindexerHardware;
    private final IndexerSpindexerIOInputsAutoLogged spindexerInputs = new IndexerSpindexerIOInputsAutoLogged();

    private final IndexerKickerIO kickerHardware;
    private final IndexerKickerIOInputsAutoLogged kickerInputs = new IndexerKickerIOInputsAutoLogged();

    public Indexer(
            IndexerSpindexerIO spindexerIO,
            IndexerKickerIO kickerIO) {
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

    public Command setIndexerState(IndexerState state) {
        return Commands.runOnce(() -> {
            indexerState = state;
        }); // TODO: Make run instead of runOnce?
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

}
