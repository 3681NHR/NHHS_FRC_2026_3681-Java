package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    
    IndexerIO io;
    IndexerIOInputsAutoLogged in = new IndexerIOInputsAutoLogged();

    public Indexer(){
        
    }

    @Override
    public void periodic(){

    }
}
