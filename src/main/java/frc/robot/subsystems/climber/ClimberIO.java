package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface ClimberIO {
    public default void setVoltage(Voltage voltage){}
    public default void updateInputs(ClimberIOInputs input){}
    /** sets the goal position for the climber. */
    public default void setSetpoint(double position){}
    public default void zeroEncoder(){}
    @AutoLog
    public class ClimberIOInputs{
        public Voltage motorVoltageOut = Volts.zero();
        public Current motorCurrentOut = Amps.zero();
        public Temperature motorTemp = Celsius.zero();
        public Distance encoderPosition = Meters.zero();
        public LinearVelocity encoderVelocity = MetersPerSecond.zero();
        public LinearVelocity climbVelocitySetpoint = MetersPerSecond.zero();
        public Distance climbPositionSetpoint = Meters.zero();
        public boolean connected = false;
        public Distance goal = Meters.zero();
        public boolean atSetpoint = false;
        public boolean openLoop = false;
    }

    
}
