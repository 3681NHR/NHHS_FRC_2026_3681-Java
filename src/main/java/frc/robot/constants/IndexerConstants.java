package frc.robot.constants;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.utils.controlWrappers.PIDGains;

public final class IndexerConstants {
    public static final int INDEXER_MOTOR_ID = -1;

    public static final AngularVelocity INDEXER_SPEED = RPM.of(0);
    public static final PIDGains.PID LAUNCHER_PID_GAINS = new PIDGains.PID(1.0,0,0.0);
}
