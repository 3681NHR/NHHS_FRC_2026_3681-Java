package frc.robot.subsystems.indexer;


import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;

public class Indexer extends SubsystemBase {
    
    IndexerIO io;
    IndexerIOInputsAutoLogged in = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(in);
        Logger.processInputs("IO/Indexer", in);
    }

    public Command run() {
        return Commands.runOnce(() -> {
            io.setGoal(IndexerConstants.INDEXER_SPEED);
        }, this);
    }

    public Command stop() {
        return Commands.runOnce(() -> {
            io.setGoal(RPM.of(0));
        }, this);
    }
}
