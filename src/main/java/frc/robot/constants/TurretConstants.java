package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.controlWrappers.PIDGains.ProfiledPID;
import frc.utils.controlWrappers.PIDGains.SimpleFF;

public final class TurretConstants {
    
    public static final int TURRET_MOTOR_ID = 50;
    public static final int TURRET_ENCODER_1_ID = 31;
    public static final int TURRET_ENCODER_2_ID = 32;

    public static final int TURRET_MAIN_GEAR_TEETH = 200;
    public static final int TURRET_ENCODER_1_GEAR_TEETH = 35;
    public static final int TURRET_ENCODER_2_GEAR_TEETH = 34;
    public static final int TURRET_MOTOR_GEAR_TEETH = 20;

    public static final Current TURRET_CURRENT_LIM = Amps.of(30);
    public static final boolean TURRET_MOTOR_INVERT = false;

    public static final Angle TURRET_ANGLE_OFFSET = Degrees.of(0);
    public static final Angle TURRET_ANGLE_LIM = Degrees.of(360);//soft limit before unwind(from center)

    public static final SimpleFF TURRET_ID_GAINS = new SimpleFF(0.15749,0.23831, 0.0087143);//gains from sysid for state space model

    public static final SimpleFF TURRET_FF_GAINS = new SimpleFF(0.2,0.25, 0.03).makeTunable("Turret FF");
    public static final ProfiledPID TURRET_PID_GAINS = new ProfiledPID(0.8,0.0,0.15,50,100).makeTunable("Turret PID");
    public static final double TURRET_THETA_COMP_FACTOR = -0.08;//offset target angle while robot is spinning

    public static final Angle TURRET_SETPOINT_TOLERANCE = Degrees.of(5);
    public static final Angle TURRET_DIVERGANCE_THRESH = Degrees.of(5);

    public static final Translation2d RED_HUB = new Translation2d(11.915, 4.034);
    public static final Translation2d[] RED_PASS = new Translation2d[]{
        new Translation2d(13, 6.5),
        new Translation2d(13, 2)
    };
    
    public static final Translation2d BLUE_HUB = new Translation2d(4.625, 4.034);
    public static final Translation2d[] BLUE_PASS = new Translation2d[]{
        new Translation2d(3.5, 6.5),
        new Translation2d(3.5, 2)
    };

    public static final Angle TURRET_LOCK_POS = Degrees.of(0.0);

    public static final Translation3d TURRET_OFFSET = new Translation3d(-.158750,0,0.298450);
    public static final Translation3d HOOD_TO_TURRET_OFFSET = new Translation3d(0.085914,0,0.141886);

    public static final SysIdRoutine.Config TURRET_SYSID_CONFIG = new SysIdRoutine.Config(
        Volts.per(Second).of(1.0), 
        Volts.of(5.0), 
        Seconds.of(5),
        (state) -> Logger.recordOutput("Turret/SysIdTestState", state.toString())
    );
}
