package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.VisionConstants.CameraConfig;

import static edu.wpi.first.units.Units.*;

public class FuelVisionConstants {

    // Camera names, must match names configured on coprocessor
    public static CameraConfig CAMERA_CONFIG = new CameraConfig(
            "det",
            "object detector",
            1.0,
            1.0,
            new Transform3d(
                Units.inchesToMeters(3.185501),
                Units.inchesToMeters(0.733652-1.0),
                Units.inchesToMeters(19.913505),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(10),
                    Units.degreesToRadians(0)
                )
            )
        );

    public static final Distance FUEL_RADIUS = Inches.of(3);

    //area of fuel on screen where area=dist(magic number for radial)
    public static final double FUEL_SIZE_BASELINE = 1.54;//TODO: tune

    public static final Time FUEL_PERSISTANCE_TIME = Seconds.of(3);
    public static final Time FUEL_POV_CLEAR_GRACE_TIME = Milliseconds.of(200);
    public static final Distance FUEL_OVERLAP_THRESH = Inches.of(2);

    public static final Angle CAMERA_CLEAR_HFOV = Degrees.of(50);
    public static final Distance MIN_CLEAR_DETECTION_DIST = Meters.of(1.0);
    public static final Distance MAX_CLEAR_DETECTION_DIST = Meters.of(5);

    public static final Distance MIN_DETECTION_DIST = Meters.of(0.5);
    public static final Distance MAX_DETECTION_DIST = Meters.of(6);

    public static final Angle CAMERA_REAL_HFOV = Degrees.of(71.2);
    public static final Distance GRID_SIZE = Inches.of(20);

    public static final Distance ROBOT_WIDTH = Inches.of(20);

    public static final int MAX_FUEL = 500;
}