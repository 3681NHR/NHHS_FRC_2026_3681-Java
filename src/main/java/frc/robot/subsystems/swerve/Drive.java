package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.FineTuneAlign;
import frc.robot.commands.playCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.Constants.RobotMode;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionEstimate;
import frc.utils.ExtraMath;
import frc.utils.LoggedField2d;
import frc.utils.PhoenixOdometryThread;
import frc.utils.SparkOdometryThread;
import frc.utils.Joystick.duelJoystickAxis;
import frc.utils.controlWrappers.PID;

import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * subsystem for Swerve drivebase using IO abstraction and a state machine
 */
public class Drive extends SubsystemBase {

    private duelJoystickAxis driverSticks;

    private Led led;

    private boolean FODEnabled = true;

    private PID angleController = new PID(
            RobotBase.isReal() ? ANGLE_PID : ANGLE_PID_SIM);

    public PPHolonomicDriveController autoController = new PPHolonomicDriveController(
            new PIDConstants(TRANS_PID.kP, TRANS_PID.kI, TRANS_PID.kD),
            new PIDConstants(AUTO_ANGLE_PID.kP, AUTO_ANGLE_PID.kI, AUTO_ANGLE_PID.kD));

    public static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine driveSysId;
    private final SysIdRoutine steerSysId;
    private final SysIdRoutine angleSysId;
    private final Vision vision;
    private LoggedField2d field = new LoggedField2d();
    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
            AlertType.kError);

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_POSITIONS);
    private Angle rawGyroRotation = Radians.of(0);
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(rawGyroRotation.in(Radians)),
            lastModulePositions, Constants.STARTING_POSE);

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO,
            Vision vision,
            duelJoystickAxis driverController,
            Led led) {
        this.vision = vision;
        this.gyroIO = gyroIO;
        this.driverSticks = driverController;
        this.led = led;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        // rotation lock pid needs to be continuous
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        SparkOdometryThread.getInstance().start();
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                this::runVelocity,
                autoController,
                PP_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Subsystems/Swerve/Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                    field.getObject("PP/activePath").setPoses(activePath);
                });

        // Configure SysId
        driveSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        DRIVE_SYSID_VRAMP,
                        DRIVE_SYSID_VSTEP,
                        DRIVE_SYSID_TIMEOUT,
                        (state) -> Logger.recordOutput("Subsystems/Swerve/Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runCharacterization(voltage), null, this));
        steerSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        TURN_SYSID_VRAMP,
                        TURN_SYSID_VSTEP,
                        TURN_SYSID_TIMEOUT,
                        (state) -> Logger.recordOutput("Subsystems/Swerve/Drive/SteerSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runSteerCharacterization(voltage), null, this));
        angleSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        Seconds.of(10),
                        (state) -> Logger.recordOutput("Subsystems/Swerve/AngleSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runAngleCharacterization(voltage.in(Volts)), null, this));

        YAGSLWidget.maxAngularVelocity = getMaxAngularSpeedRadPerSec();
        YAGSLWidget.maxSpeed = getMaxLinearSpeedMetersPerSec();
        YAGSLWidget.moduleCount = 4;
        YAGSLWidget.sizeFrontBack = LENGTH.in(Meters);
        YAGSLWidget.sizeLeftRight = WIDTH.in(Meters);
        YAGSLWidget.wheelLocations = new double[8];

        angleController.setTolerance(AUTO_ALIGN_ANGLE_MAX_OFFSET.in(Radians));

        for (int i = 0; i > MODULE_POSITIONS.length; i += 2) {
            Translation2d t = MODULE_POSITIONS[i];
            YAGSLWidget.wheelLocations[i * 2] = t.getX();
            YAGSLWidget.wheelLocations[(i * 2) + 1] = t.getY();
        }
    }
    public void setCallback(){
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Subsystems/Swerve/Odometry/TrajectorySetpoint", targetPose);
                    field.getObject("PP/targetpose").setPoses(targetPose);
                });
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        //update and log gyro and module IOs
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("IO/Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        if (USE_VISION) {
            for (VisionEstimate e : vision.getPose()) {
                poseEstimator.addVisionMeasurement(
                        e.pose, e.timestampSeconds, e.visionMeasurementStdDevs);
            }
        }
        odometryLock.unlock();

        Logger.recordOutput("Subsystems/Swerve/tilt", ExtraMath.getTip(gyroInputs.angle));

        Logger.recordOutput("Subsystems/Swerve/CurrentCommand",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "none");

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for(int i=0; i<4; i++){
                modules[i].stop();
            }
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                                - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = Radians.of(gyroInputs.odometryYawPositions[i]);
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(Radians.of(twist.dtheta));
            }
            Logger.recordOutput("Subsystems/Swerve/Odometry/timestamp", sampleTimestamps[i]);
            Logger.recordOutput("Subsystems/Swerve/Odometry/position", modulePositions);

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], new Rotation2d(rawGyroRotation.in(Radians)), modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.MODE != RobotMode.SIM);

        //update swerve widget
        YAGSLWidget.measuredStatesObj = getModuleStates();
        YAGSLWidget.measuredChassisSpeedsObj = getChassisSpeeds();
        YAGSLWidget.robotRotationObj = getRotation();
        YAGSLWidget.updateData();

        field.setRobotPose(getPose());

        SmartDashboard.putData("field", field);
    }

    public void setFOD(boolean fod) {
        this.FODEnabled = fod;
    }

    @AutoLogOutput(key="Subsystems/Swerve/FOD enabled")
    public boolean getFOD(){
        return FODEnabled;
    }

    private ChassisSpeeds getSpeedsFromController() {

        ChassisSpeeds speed = new ChassisSpeeds();
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                speed = new ChassisSpeeds(
                        -driverSticks.ly.getAsDouble() * getMaxLinearSpeedMetersPerSec(),
                        -driverSticks.lx.getAsDouble() * getMaxLinearSpeedMetersPerSec(),
                        driverSticks.rx.getAsDouble() * getMaxAngularSpeedRadPerSec());
            } else {
                speed = new ChassisSpeeds(
                        driverSticks.ly.getAsDouble() * getMaxLinearSpeedMetersPerSec(),
                        driverSticks.lx.getAsDouble() * getMaxLinearSpeedMetersPerSec(),
                        driverSticks.rx.getAsDouble() * getMaxAngularSpeedRadPerSec());
            }
        }
        double skew = speed.omegaRadiansPerSecond * ANGULAR_VELOCITY_COEFFICIENT;

        return ChassisSpeeds.fromFieldRelativeSpeeds(speed, getRotation().plus(new Rotation2d(skew)));
    }

    private ChassisSpeeds getTranslationalSpeedsFromController(double angularVelocity) {

        ChassisSpeeds speed = new ChassisSpeeds();
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                speed = new ChassisSpeeds(
                        -driverSticks.ly.getAsDouble() * getMaxLinearSpeedMetersPerSec(),
                        -driverSticks.lx.getAsDouble() * getMaxLinearSpeedMetersPerSec(),
                        angularVelocity);
            } else {
                speed = new ChassisSpeeds(
                        driverSticks.ly.getAsDouble() * getMaxLinearSpeedMetersPerSec(),
                        driverSticks.lx.getAsDouble() * getMaxLinearSpeedMetersPerSec(),
                        angularVelocity);
            }
        }
        double skew = speed.omegaRadiansPerSecond * ANGULAR_VELOCITY_COEFFICIENT;

        return ChassisSpeeds.fromFieldRelativeSpeeds(speed, getRotation().plus(new Rotation2d(skew)));
    }

    public Command rotationLock(DoubleSupplier headingRad){
        return new InstantCommand(() -> {
            led.rotLock = true;
            angleController.reset();
        }).andThen(Commands.run(() -> {
            
            ChassisSpeeds speeds = getTranslationalSpeedsFromController(
                MathUtil.clamp(angleController.calculate(getRotation().getRadians(), headingRad.getAsDouble()),
                        -ANGLE_MAX_VELOCITY.in(RadiansPerSecond), ANGLE_MAX_VELOCITY.in(RadiansPerSecond)));

            Logger.recordOutput("Subsystems/Swerve/Rotation lock/Target angle", headingRad.getAsDouble());
            Logger.recordOutput("Subsystems/Swerve/Rotation lock/Angle PID out", speeds.omegaRadiansPerSecond);

            runVelocity(speeds);

        }, this))
        .until(() -> angleController.atSetpoint())
        .finallyDo(() -> {
            Logger.recordOutput("Subsystems/Swerve/Rotation lock/Target angle", Double.NaN);
            Logger.recordOutput("Subsystems/Swerve/Rotation lock/Angle PID out", Double.NaN);
            led.rotLock = false;
        })
        .withName("Rotation lock");
    }

    public Command TeleopDrive(){
        return Commands.run(() -> {
            if (FODEnabled) {
                runVelocity(getSpeedsFromController());
            } else {
                runVelocity(new ChassisSpeeds(
                        driverSticks.ly.getAsDouble() * getMaxLinearSpeedMetersPerSec(),
                        driverSticks.lx.getAsDouble() * getMaxLinearSpeedMetersPerSec(),
                        driverSticks.rx.getAsDouble() * getMaxAngularSpeedRadPerSec()));
            }
        }, this)
        .withName("Teleop drive");
    }

    public Command getAutoAlign(Supplier<Pose2d> p) {
        return driveToPoseAuto(p)
                .withTimeout(2)
                .finallyDo(() -> {
                    led.aligningReef = false;
                });
    }

    public Command followPath(PathPlannerPath path) {
        return AutoBuilder.followPath(path)
                .withName("Follow path: " + path.name);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.EVENT_LOOP_TIME);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, getMaxLinearSpeedMetersPerSec());

        YAGSLWidget.desiredChassisSpeedsObj = discreteSpeeds;
        // Log unoptimized setpoints
        Logger.recordOutput("Subsystems/Swerve/Drive/SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("Subsystems/Swerve/Drive/SwerveChassisSpeeds/Setpoints", discreteSpeeds);
        Logger.recordOutput("Subsystems/Swerve/Drive/SwerveChassisSpeeds/SetpointAngularVel", discreteSpeeds.omegaRadiansPerSecond);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        YAGSLWidget.desiredStatesObj = setpointStates;
        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("Subsystems/Swerve/Drive/SwerveStates/SetpointsOptimized", setpointStates);
    }


    /** Stops the drive. */
    public void stop() {
        for (int i = 0; i < 4; i++) {
            modules[i].stop();
        }
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will
     * return to their normal orientations the next time a nonzero velocity is
     * requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = MODULE_POSITIONS[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
        runVelocity(new ChassisSpeeds());
    }


    public void resetGyro(double headingRad) {
        poseEstimator.resetPose(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d(headingRad)));
    }


    Command follow = new InstantCommand();
    Command play = new playCommand(() -> follow);

    private Command driveToPoseAuto(Supplier<Pose2d> p) {
        return new InstantCommand(() -> {
            Pose2d end = new Pose2d(p.get().getTranslation(), p.get().getRotation().rotateBy(Rotation2d.kCCW_90deg));
            Pose2d start = new Pose2d(getPose().getTranslation(),
                    getPathVelocityHeading(getFieldChassisSpeeds(), p.get()));

            List<Waypoint> points = PathPlannerPath.waypointsFromPoses(start, end);

            PathConstraints constraints = new PathConstraints(DriveConstants.MAX_SPEED_PP, DriveConstants.MAX_ACCEL_PP,
                    DriveConstants.MAX_ANGLE_SPEED_PP, DriveConstants.MAX_ANGLE_ACCEL_PP);
            PathPlannerPath path = new PathPlannerPath(points, constraints,
                    new IdealStartingState(getSpeed(), getPose().getRotation()),
                    new GoalEndState(0, p.get().getRotation()));
            path.preventFlipping = true;

            led.alignInPos = false;

            follow = AutoBuilder.followPath(path);

        }).andThen(new playCommand(() -> follow).withName("Follow autogenerated path"), new FineTuneAlign(p, this, led).withName("Fine tune alignment"))
        .alongWith(new InstantCommand(() -> {
            led.aligningReef = true;
        }));
    }

    /**
     * 
     * @param cs field relative chassis speeds
     * @return
     */
    private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target) {
        if (getSpeed() < 0.25) {
            Logger.recordOutput("Subsystems/Swerve/Align/approach", "straight line");
            var diff = target.getTranslation().minus(getPose().getTranslation());
            Logger.recordOutput("Subsystems/Swerve/Align/Calc/x", diff.getX());
            Logger.recordOutput("Subsystems/Swerve/Align/Calc/y", diff.getY());
            Logger.recordOutput("Subsystems/Swerve/Align/Calc/dir", diff.getAngle());

            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();
        }

        Logger.recordOutput("Subsystems/Swerve/Align/approach", "velocity comp");

        var rotation = new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);

        Logger.recordOutput("Subsystems/Swerve/Align/Calc/x", cs.vxMetersPerSecond);
        Logger.recordOutput("Subsystems/Swerve/Align/Calc/y", cs.vyMetersPerSecond);
        Logger.recordOutput("Subsystems/Swerve/Align/Calc/dir", rotation);

        return rotation;
    }

    public AngularVelocity getAngulerVelocity() {
        return gyroInputs.yawVelocity;
    }

    @AutoLogOutput(key = "Subsystems/Swerve/Speed")
    public double getSpeed() {
        return new Translation2d(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond).getNorm();
    }

    public Rotation2d getVelocityDir() {
        return new Rotation2d(Math.atan2(getChassisSpeeds().vyMetersPerSecond, getChassisSpeeds().vxMetersPerSecond));
    }
    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(Volts.of(0.0)))
                .withTimeout(1.0)
                .andThen(driveSysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(Volts.of(0.0))).withTimeout(1.0).andThen(driveSysId.dynamic(direction));
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command steerSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runSteerCharacterization(Volts.of(0.0)))
                .withTimeout(1.0)
                .andThen(steerSysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command steerSysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runSteerCharacterization(Volts.of(0.0))).withTimeout(1.0).andThen(steerSysId.dynamic(direction));
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command angleSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runAngleCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(angleSysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command angleSysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runAngleCharacterization(0.0)).withTimeout(1.0).andThen(angleSysId.dynamic(direction));
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules.
     */
    @AutoLogOutput(key = "Subsystems/Swerve/Drive/SwerveStates/Measured")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }

        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the
     * modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "Subsystems/Swerve/ChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "Subsystems/Swerve/ChassisSpeeds/Field Relative")
    private ChassisSpeeds getFieldChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(getModuleStates()), getRotation());
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Subsystems/Swerve/Odometry/Robot Pose")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(new Rotation2d(rawGyroRotation.in(Radians)), getModulePositions(), pose);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return MAX_SPEED_PP.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        // return MAX_SPEED_PP.in(MetersPerSecond) / RADIUS.in(Meters);
        return MAX_ANGLE_SPEED_PP.in(RadiansPerSecond);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(Voltage output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** spins modules */
    public void runSteerCharacterization(Voltage output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runSteerCharacterization(output);
        }
    }

    /** spins robot */
    public void runAngleCharacterization(double output) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(new ChassisSpeeds(0, 0, 1), Constants.EVENT_LOOP_TIME);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(Volts.of(output), Radians.of(setpointStates[i].angle.getRadians()));
        }
    }
}