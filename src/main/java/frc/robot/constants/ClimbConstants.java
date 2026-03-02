package frc.robot.constants;

import frc.utils.controlWrappers.PIDGains;

public class ClimbConstants {
    public static final int MOTOR_ID = 5;
    public static final boolean INVERTED = false;
    public static final double EXTEND_POSITION = 1.0;
    public static final PIDGains.ProfiledPID CLIMB_PID_GAINS = new PIDGains.ProfiledPID(1, 1, 1, 100, 100).makeTunable("Climber PID");
    public static final PIDGains.GravityFF FF = new PIDGains.GravityFF(0, 0, 0, 0).makeTunable("Climber FF");
    public static final PIDGains.GravityFF CLIMB_ID_GAINS = new PIDGains.GravityFF(0.1, 0.1, 0.1, 0.01);
    private static final double GEAR_RATIO = 80.0;
    private static final double SPOOL_CIRC = Math.PI * 1.0; // get actual circumference later
    public static final double POSITION_CONVERSION_FACTOR = SPOOL_CIRC/GEAR_RATIO;
    public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR/60; // RPM to RPS
}