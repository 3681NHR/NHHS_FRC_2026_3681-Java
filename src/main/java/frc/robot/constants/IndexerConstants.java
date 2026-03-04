package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.utils.controlWrappers.PIDGains;

public final class IndexerConstants {
    public static final int INDEXER_MOTOR_ID = -1;
    public static final boolean INVERT = false;

    public static final AngularVelocity INDEXER_SPEED = RPM.of(0);
    public static final PIDGains.PID INDEXER_PID_GAINS = new PIDGains.PID(1.0,0,0.0);
    public static final PIDGains.SimpleFF INDEXER_FF_GAINS = new PIDGains.SimpleFF(0, 0, 0);

    public static final Current CURRENT_LIMIT = Amps.of(40);
    public static final Temperature MAX_TEMP = Celsius.of(40);
}
