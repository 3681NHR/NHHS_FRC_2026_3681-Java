package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kelvin;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface IndexerIO {
    
    public default void updateInputs(IndexerIOInputs input) {}
    public default void setGoal(AngularVelocity goal) {}
    public default void setVoltage(Voltage volts) {}
    public default void setOpenLoop(boolean openLoop) {}
    
    @AutoLog
    public class IndexerIOInputs{
        public AngularVelocity speed = RadiansPerSecond.of(0.0);

        public Voltage motorVoltageOut = Volts.of(0);
        public Current motorCurrentOut = Amps.of(0);
        public Temperature motorTemp = Kelvin.of(0);

        public AngularVelocity goal = RadiansPerSecond.of(0);

        public boolean openLoop = false;
    }
}
