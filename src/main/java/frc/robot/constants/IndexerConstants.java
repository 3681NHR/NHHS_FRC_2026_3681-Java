package frc.robot.constants;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public final class IndexerConstants {
    public static final int INDEXER_MOTOR_ID = 43;

    public static final boolean INDEXER_MOTOR_INVERT = false;
    public static final Current INDEXER_MAX_CURRENT = Amps.of(40);

    public static final Voltage INDEXER_FEED_VOLTAGE = Volts.of(10);

    public static final double POSITION_CONVERSION_FACTOR = 1.0;
    public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR*60;
}
