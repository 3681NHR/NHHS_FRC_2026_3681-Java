package frc.robot.subsystems.indexer;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public Command voltageControl(Supplier<Voltage> volt){
        return Commands.run(() -> {
            io.setVout(volt.get());
        }, this)
        .withName("Voltage Control");
    }
}
