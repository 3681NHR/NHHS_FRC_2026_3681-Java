package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltOutpost;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class SimFuelManager {
    private static SimFuelManager instance;

    private SimFuelManager(){}

    public synchronized static SimFuelManager getInstance(){
        if(instance == null){
            instance = new SimFuelManager();
        }
        return instance;
    }
    public int capacity;

    public IntakeSimulation intake;
}