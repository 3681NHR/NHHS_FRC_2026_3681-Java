package frc.robot.constants;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

    public enum RobotMode {
        REAL,
        SIM,
        REPLAY
    }

    public static RobotMode SIM_MODE = RobotMode.SIM;// set to REPLAY to replay log files
    public static RobotMode MODE = RobotBase.isReal() ? RobotMode.REAL : SIM_MODE;

    public static final Time AUTO_TIME = Seconds.of(20);
    public static final Time TELEOP_TIME = Seconds.of(140);// 2:20
    public static final Time ENDGAME_TIME = Seconds.of(30);// time remaining in teleop when endgame starts
    public static final double EVENT_LOOP_TIME = 0.02; // 20ms
    // default robot pose
    public static final Pose2d STARTING_POSE = new Pose2d(new Translation2d(8.75, 4), Rotation2d.fromDegrees(0));

    public static final Mass ROBOT_MASS = Pounds.of(115);
    public static final Time LOOP_TIME = Milliseconds.of(0.13); // 20ms + 110ms spark max velocity lag
    // 3742, thanks Mr. B
    public static final int ELASTIC_LAYOUT_PORT = 3742;
    public static class OperatorConstants {
        // Joystick Deadbands, radial from center
        public static final double LEFT_DEADBAND = 0.1;
        public static final double RIGHT_DEADBAND = 0.15;

        // stick curvature
        public static final double TRANSLATION_CURVE = 2;
        public static final double ROTATION_CURVE = 2;

        // usb ports of controllers, remember to assign controllers to their respective
        // ports in driverstation
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static class drive {
        //if robot should have field oriented drive enabled on start
        public static final boolean STARTING_FOD = true;
    }

    public static class DPAD {
        public static final int UP = 0;
        public static final int RIGHT = 90;
        public static final int DOWN = 180;
        public static final int LEFT = 270;
        public static final int NONE = -1;
        public static final int RIGHT_UP = 45;
        public static final int RIGHT_DOWN = 135;
        public static final int LEFT_DOWN = 225;
        public static final int LEFT_UP = 315;
        private DPAD() {
            throw new UnsupportedOperationException("This is a utility class!");
        }
    }

    public static final int SPARKMAX_TARGET_FIRMWARE = 436273156;
    public static final int TALONFX_TARGET_FIRMWARE = 436273152;
}
