/* I wrote this robot code with furry paws on. Just thought I would mention that. -yarden*/

package frc.robot;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.autos.AutoGenerator;
import frc.robot.oldautos.AutoChooser;
import frc.robot.commands.SwerveWheelCharacterization;
import frc.robot.constants.*;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.SOTMSolver;
import frc.robot.subsystems.SimFuelManager;
import frc.robot.subsystems.LaunchLUT;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.fuelVision.FuelVision;
import frc.robot.subsystems.fuelVision.FuelVisionIO;
import frc.robot.subsystems.fuelVision.FuelVisionIOPhoton;
import frc.robot.subsystems.fuelVision.FuelVisionIOPhotonSim;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOReal;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherIO;
import frc.robot.subsystems.launcher.LauncherIOReal;
import frc.robot.subsystems.launcher.LauncherIOSim;
import frc.robot.subsystems.physButtons.ButtonIO;
import frc.robot.subsystems.physButtons.ButtonIODIO;
import frc.robot.subsystems.physButtons.ButtonIOSim;
import frc.robot.subsystems.physButtons.Buttons;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOCrackingSpark;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOPhoton;
import frc.robot.subsystems.vision.CameraIOPhotonSim;
import frc.robot.subsystems.vision.Vision;
import frc.utils.*;
import frc.utils.Joystick;
import frc.utils.rumble.*;
import frc.utils.Joystick.DuelJoystickAxis;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.Constants.MODE;
import static frc.robot.constants.HoodConstants.HOOD_MIN_ANGLE;
import static frc.robot.constants.IntakeConstants.INTAKE_OFFSET;
import static frc.robot.constants.TurretConstants.*;

import static frc.utils.ControllerMap.*;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.Arrays;
import java.util.Random;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIOReal;
import frc.robot.subsystems.kicker.KickerIOSim;
import frc.robot.constants.Constants.RobotMode;
import org.opencv.core.Mat;

public class RobotContainer {

    private double currentStartTimestamp = 0.0;
    private Angle hoodSetpoint = Degrees.zero();
    private AngularVelocity launcherSetpoint = RPM.zero();
    private Distance distanceToHub = Meters.zero();

    private Random rand = new Random();
    private double gauss = 0.4;

    private boolean isLutInProgress = false;
    private final LoggedDashboardChooser<Command> sysidChooser = new LoggedDashboardChooser<Command>("sysid auto chooser");

    private DriveTrainSimulationConfig driveTrainSimulationConfig;
    private SwerveDriveSimulation driveSim;
    private Drive drive;
    private Vision vision;
    @SuppressWarnings("unused")
    private FuelVision fuelVision;
    private Turret turret;
    private Launcher launcher;
    private Climber climber;
    private Hood hood;
    private Intake intake;
    private Kicker kicker;
    private Buttons buttons;
    private Indexer indexer;

    @SuppressWarnings("unused")
    private Led led;
    private boolean manual = true;

    boolean hubTrack = false;

    private final XboxController driverController = new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final XboxController operatorController = new XboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    private final LoggedNetworkBoolean resetOdometry = new LoggedNetworkBoolean("Debug/Reset Odometry", false);
    private final LoggedNetworkBoolean TStop = new LoggedNetworkBoolean("Overrides/Paralyze turret", false);
    private final LoggedNetworkBoolean autoTrench = new LoggedNetworkBoolean("Overrides/use autotrench", true);
    private final LoggedNetworkBoolean shiftLock = new LoggedNetworkBoolean("Overrides/use shift tracker", true);
    private AutoChooser autoChooser;

    private final RumbleHandler rumbler = new RumbleHandler(driverController);
    private final RumbleHandler opRumbler = new RumbleHandler(operatorController);

    private Translation2d target = new Translation2d();

    AprilTagFieldLayout apriltagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
    private final Alert operatorDisconnected = new Alert("Operator controller disconnected (port 1).",
            AlertType.kWarning);

    private final LoggedNetworkBoolean useLead = new LoggedNetworkBoolean("Overrides/Enable SOTM", true);
    private final LoggedNetworkBoolean forceFeed = new LoggedNetworkBoolean("Overrides/Force Feed", false);

    private final LoggedNetworkNumber manHoodDegrees = new LoggedNetworkNumber("Manual/Hood angle degrees", HOOD_MIN_ANGLE.in(Degrees));
    private final LoggedNetworkNumber manShooterRPM = new LoggedNetworkNumber("Manual/Shooter speed RPM", 0);
    private final LoggedNetworkNumber manTurretDegrees = new LoggedNetworkNumber("Manual/Turret angle degrees", 0);

    private final Debouncer readyDebounce = new Debouncer(0.2);

    private DuelJoystickAxis driverSticks;

    private final AutoGenerator generator;

    public RobotContainer() {
        frc.robot.autos.Path.container = this;
        generator = new AutoGenerator(this);
        try {
            // load test field layout for camera offset calculation, do not use otherwise
            // e = new AprilTagFieldLayout(Filesystem.getDeployDirectory().getAbsolutePath()
            // + "/test_field.json");
        } catch (Exception ex) {
        }

        Logger.recordOutput("AScope/zeroPose", new Pose3d());

        // we use our own warnings for joysticks
        DriverStation.silenceJoystickConnectionWarning(true);

        if (RobotBase.isSimulation()) {
            // maplesim setup
            driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
                    .withGyro(COTS.ofPigeon2())
                    .withSwerveModule(COTS.ofMark4i(
                            DCMotor.getKrakenX60(1),
                            DCMotor.getNEO(1),
                            COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                            2))
                    .withTrackLengthTrackWidth(DriveConstants.LENGTH, DriveConstants.WIDTH)
                    .withBumperSize(Inches.of(31), Inches.of(33));

            driveSim = new SwerveDriveSimulation(driveTrainSimulationConfig, Constants.STARTING_POSE);
            SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
            //FIXME: this line is why your sim sucks
        //    ((Arena2026Rebuilt) SimulatedArena.getInstance()).setEfficiencyMode(false);
        }

        // process driver controls(radial deadzone, curve, trigger slowdown, and
        // inversion)
        driverSticks = new DuelJoystickAxis(
                () -> ExtraMath.processInput(
                        Joystick.deadzone(OperatorConstants.LEFT_DEADBAND,
                                        driverController.getRawAxis(LEFT_STICK_X), driverController.getRawAxis(LEFT_STICK_Y))
                                .getX(),
                        -1.0,
                        OperatorConstants.TRANSLATION_CURVE, 0.0),
                () -> ExtraMath.processInput(
                        Joystick.deadzone(OperatorConstants.LEFT_DEADBAND,
                                        driverController.getRawAxis(LEFT_STICK_X), driverController.getRawAxis(LEFT_STICK_Y))
                                .getY(),
                        -1.0,
                        OperatorConstants.TRANSLATION_CURVE, 0.0),
                () -> ExtraMath.processInput(
                        Joystick.deadzone(OperatorConstants.RIGHT_DEADBAND,
                                        driverController.getRawAxis(RIGHT_STICK_X), driverController.getRawAxis(RIGHT_STICK_Y))
                                .getX(),
                        -0.75,
                        OperatorConstants.ROTATION_CURVE, 0.0),
                () -> ExtraMath.processInput(
                        Joystick.deadzone(OperatorConstants.RIGHT_DEADBAND,
                                        driverController.getRawAxis(RIGHT_STICK_X), driverController.getRawAxis(RIGHT_STICK_Y))
                                .getY(),
                        -1.0,
                        OperatorConstants.ROTATION_CURVE, 0.0));

        switch (MODE) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                vision = new Vision(
                        apriltagLayout,
                        new CameraIOPhoton(apriltagLayout, VisionConstants.CAMERA_CONFIGS[0]),
                        new CameraIOPhoton(apriltagLayout, VisionConstants.CAMERA_CONFIGS[1]),
                        new CameraIOPhoton(apriltagLayout, VisionConstants.CAMERA_CONFIGS[2]),
                        new CameraIOPhoton(apriltagLayout, VisionConstants.CAMERA_CONFIGS[3]));
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOCrackingSpark(0),
                        new ModuleIOCrackingSpark(1),
                        new ModuleIOCrackingSpark(2),
                        new ModuleIOCrackingSpark(3),
                        vision,
                        driverSticks);
                SOTMSolver.getInstance().setDrive(drive);
                SOTMSolver.getInstance().calculate();

                fuelVision = new FuelVision(new FuelVisionIOPhoton(FuelVisionConstants.CAMERA_CONFIG), drive::getPose);

                turret = new Turret(new TurretIOReal(), drive);
                intake = new Intake(new IntakeIOReal());
                launcher = new Launcher(new LauncherIOReal());
                hood = new Hood(new HoodIOReal());
                climber = new Climber(new ClimberIO() {
                });//FIXME
                kicker = new Kicker(new KickerIOReal());
                buttons = new Buttons(new ButtonIODIO(0));
                indexer = new Indexer(new IndexerIOReal());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                vision = new Vision(
                        apriltagLayout,
                        new CameraIOPhotonSim(apriltagLayout, VisionConstants.CAMERA_CONFIGS[0],
                                driveSim::getSimulatedDriveTrainPose),
                        new CameraIOPhotonSim(apriltagLayout, VisionConstants.CAMERA_CONFIGS[1],
                                driveSim::getSimulatedDriveTrainPose),
                        new CameraIOPhotonSim(apriltagLayout, VisionConstants.CAMERA_CONFIGS[2],
                                driveSim::getSimulatedDriveTrainPose),
                        new CameraIOPhotonSim(apriltagLayout, VisionConstants.CAMERA_CONFIGS[3],
                                driveSim::getSimulatedDriveTrainPose));
                if (driveSim != null) {
                    drive = new Drive(
                            new GyroIOSim(driveSim.getGyroSimulation()) {
                            },
                            new ModuleIOSim(driveSim.getModules()[0]),
                            new ModuleIOSim(driveSim.getModules()[1]),
                            new ModuleIOSim(driveSim.getModules()[2]),
                            new ModuleIOSim(driveSim.getModules()[3]),
                            vision,
                            driverSticks);
                    SOTMSolver.getInstance().setDrive(drive);
                    SOTMSolver.getInstance().calculate();
                
                     fuelVision = new FuelVision(new FuelVisionIOPhotonSim(FuelVisionConstants.CAMERA_CONFIG, driveSim::getSimulatedDriveTrainPose), drive::getPose);
//                    fuelVision = new FuelVision(new FuelVisionIO(){}, drive::getPose);
                    intake = new Intake(new IntakeIOSim(driveSim));
                    turret = new Turret(new TurretIOSim(), drive);
                }
                launcher = new Launcher(new LauncherIOSim());
                hood = new Hood(new HoodIOSim());
                climber = new Climber(new ClimberIOSim());
                kicker = new Kicker(new KickerIOSim());
                buttons = new Buttons(new ButtonIOSim(() -> false));
                indexer = new Indexer(new IndexerIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations for replay
                vision = new Vision(
                        apriltagLayout,
                        new CameraIO() {
                        },
                        new CameraIO() {
                        },
                        new CameraIO() {
                        },
                        new CameraIO() {
                        });
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        vision,
                        driverSticks);
                SOTMSolver.getInstance().setDrive(drive);
                SOTMSolver.getInstance().calculate();

                fuelVision = new FuelVision(new FuelVisionIO(){}, drive::getPose);
                intake = new Intake(new IntakeIO(){});
                turret = new Turret(new TurretIO(){}, drive);

                launcher = new Launcher(new LauncherIO(){});
                hood = new Hood(new HoodIO(){});
                climber = new Climber(new ClimberIO(){});
                kicker = new Kicker(new KickerIO(){});
                buttons = new Buttons(new ButtonIO(){});
                indexer = new Indexer(new IndexerIO(){});
                break;
        }
        led = new Led(launcher, hood, turret, drive, () -> manual);

        // build pathplanner autos and put in dashboard
        // autoChooser = new LoggedDashboardChooser<>("Auto Choices",
        // AutoBuilder.buildAutoChooser());

        configureBindings();

        autoChooser = new AutoChooser(this);
        sysidChooser.addDefaultOption("none", null);
        sysidChooser.addOption("drive sysid quasistatic forward", drive.sysIdQuasistatic(Direction.kForward));
        sysidChooser.addOption("drive sysid quasistatic reverse", drive.sysIdQuasistatic(Direction.kReverse));
        sysidChooser.addOption("drive sysid dynamic forward", drive.sysIdDynamic(Direction.kForward));
        sysidChooser.addOption("drive sysid dynamic reverse", drive.sysIdDynamic(Direction.kReverse));

        sysidChooser.addOption("steer sysid quasistatic forward", drive.steerSysIdQuasistatic(Direction.kForward));
        sysidChooser.addOption("steer sysid quasistatic reverse", drive.steerSysIdQuasistatic(Direction.kReverse));
        sysidChooser.addOption("steer sysid dynamic forward", drive.steerSysIdDynamic(Direction.kForward));
        sysidChooser.addOption("steer sysid dynamic reverse", drive.steerSysIdDynamic(Direction.kReverse));

        sysidChooser.addOption("angle sysid quasistatic forward", drive.angleSysIdQuasistatic(Direction.kForward));
        sysidChooser.addOption("angle sysid quasistatic reverse", drive.angleSysIdQuasistatic(Direction.kReverse));
        sysidChooser.addOption("angle sysid dynamic forward", drive.angleSysIdDynamic(Direction.kForward));
        sysidChooser.addOption("angle sysid dynamic reverse", drive.angleSysIdDynamic(Direction.kReverse));
        sysidChooser.addOption("swerve wheel radius char", new SwerveWheelCharacterization(drive));
        sysidChooser.addOption("turret sysid quasistatic forward", turret.sysidQuasistatic(false));
        sysidChooser.addOption("turret sysid quasistatic reverse", turret.sysidQuasistatic(true));
        sysidChooser.addOption("turret sysid dynamic forward", turret.sysidDynamic(false));
        sysidChooser.addOption("turret sysid dynamic reverse", turret.sysidDynamic(true));

        sysidChooser.addOption("launcher sysid quasistatic forward", launcher.sysidQuasistatic(false));
        sysidChooser.addOption("launcher sysid quasistatic reverse", launcher.sysidQuasistatic(true));
        sysidChooser.addOption("launcher sysid dynamic forward", launcher.sysidDynamic(false));
        sysidChooser.addOption("launcher sysid dynamic reverse", launcher.sysidDynamic(true));

        turret.setDefaultCommand(
                turret.manPos(turret::getAngle, false).ignoringDisable(true));
        launcher.setDefaultCommand(
                launcher.voltageControl(() -> Volts.of(0))
        );
        kicker.setDefaultCommand(kicker.hold().ignoringDisable(true));
        drive.setDefaultCommand(drive.teleopDrive().ignoringDisable(true));
        hood.setDefaultCommand(hood.instantPositionControl(() -> HOOD_MIN_ANGLE).ignoringDisable(true));
        climber.setDefaultCommand(climber.voltageControl(Volts::zero).ignoringDisable(true));
        indexer.setDefaultCommand(indexer.stop().ignoringDisable(true));
    }

    private void configureBindings() {
        new Trigger(() -> driverController.getRawButton(11) && MODE == RobotMode.SIM).onTrue(new InstantCommand( () -> {
            ((Arena2026Rebuilt) SimulatedArena.getInstance()).outpostDump(AllianceUtility.getAlliance() == Alliance.Blue);
        }));

        new Trigger(TStop).whileTrue(
                turret.stop().repeatedly().ignoringDisable(true).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        // reset odometry dashboard button
        resetOdometry.set(false);
        new Trigger(resetOdometry::get).onTrue(new InstantCommand(() -> {
            resetOdometry.set(false);
            drive.setPose(Constants.STARTING_POSE);
        }));

        // send haptic command when 5 seconds are left in shift
        new Trigger(() -> ShiftTracker.getTimeLeftInShift() < 5).onTrue(new InstantCommand(() -> {
            rumbler.overrideQue(RumblePreset.TAP.load());
            opRumbler.overrideQue(RumblePreset.TAP.load());
        }));

        // move wheels to X, makes robot hard to push
        new Trigger(() -> driverController.getRawButton(LOGO_RIGHT)).whileTrue(new InstantCommand(() -> {
            drive.stopWithX();
        }, drive).repeatedly());

        // reset gyro angle
        new Trigger(() -> driverController.getRawButton(LOGO_LEFT)).onTrue(new InstantCommand(() -> {
            drive.resetGyro(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? Math.PI : 0);
            rumbler.overrideQue(RumblePreset.TAP.load());
        }));

        // toggle field oriented driving
        new Trigger(() -> driverController.getRawButton(LEFT_STICK_BUTTON)).onTrue(new InstantCommand(() -> {
            drive.setFOD(!drive.getFOD());
            rumbler.overrideQue(RumblePreset.TAP.load());
        }));
        new Trigger(() -> driverController.getRawAxis(RIGHT_TRIGGER) > 0.2).whileTrue(hood.go());
        new Trigger(() -> driverController.getRawAxis(RIGHT_TRIGGER) > 0.7)
                .whileTrue(fire());
//                .onFalse(hood.positionControl(() -> HOOD_MIN_ANGLE));
        // intake :3
        new Trigger(() -> driverController.getRawAxis(LEFT_TRIGGER) > 0.5)
                .whileTrue(intake.intake());

        // force teleop drive
        new Trigger(() -> driverController.getPOV() == UP).onTrue(drive.teleopDrive());

        new Trigger(() -> driverController.getRawButton(X)).whileTrue(//lower hood
                hood.positionControl(() -> HOOD_MIN_ANGLE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        // toggle auto track command
        new Trigger(() -> driverController.getRawButton(B)).onTrue(
                getTrackCommand()
        );

        //set turret to preset angle mode
        new Trigger(() -> driverController.getRawButton(A)).onTrue(
            getManShooterCommand()
        );
        new Trigger(() -> driverController.getRawButton(Y)).onTrue(
                new HiddenConditionalCommand(saveLutEntry(), startLutTimer(), () -> isLutInProgress)
        );
        new Trigger(() -> driverController.getPOV() == RIGHT).onTrue(hood.home());
        new Trigger(() -> driverController.getPOV() == LEFT).onTrue(climber.home());

        new Trigger(() -> driverController.getRawButton(RB)).whileTrue(Commands.parallel(kicker.reverse(), indexer.reverse()));
        new Trigger(() -> driverController.getRawButton(LB)).whileTrue(Commands.parallel(intake.outtake()));

        new Trigger(() -> driverController.getPOV() == DOWN).onTrue(climber.toggle());

        new Trigger(() -> (inTrench() && autoTrench.getAsBoolean() && !DriverStation.isAutonomous()) || driverController.getRawButton(X)).whileTrue(
                drive.TrenchAlignDrive()
                    .alongWith(hood.instantPositionControl(() -> HOOD_MIN_ANGLE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                    ).withName("trench mode")
        );
        //trench mode in auto - wont cancel path following
//        new Trigger(() -> (inTrench() && autoTrench.getAsBoolean() && DriverStation.isAutonomous())).whileTrue(
//                hood.instantPositionControl(() -> HOOD_MIN_ANGLE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming
//                ).withName("trench mode(auto)")
//        );
//        .onFalse(
//                new HiddenConditionalCommand(
//                        getManShooterCommand(),
//                        getTrackCommand(),
//                        () -> manual
//                )
//        );

        new Trigger(() -> buttons.get(0) && !DriverStation.isEnabled()).onTrue(hood.forceHome());
    }



    public void periodic() {
        rumbler.update(Constants.EVENT_LOOP_TIME);
        ZoneManager.updateRobotPose(drive.getPose());
        PIDTuner.updateTunables();
        SOTMSolver.getInstance().setTarget(target);
        driverDisconnected.set(!driverController.isConnected());
        operatorDisconnected.set(!operatorController.isConnected());

        Translation2d hub = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? RED_HUB
                : BLUE_HUB;
        Translation2d pass = drive.getPose().getTranslation().nearest(Arrays
                .asList(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? RED_PASS : BLUE_PASS));
        hubTrack = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                ? drive.getPose().getX() > 12
                : drive.getPose().getX() < 4.5);
        Logger.recordOutput("Subsystems/Turret/track/tracking hub", hubTrack);
        target = hubTrack ? hub : pass;

        autoChooser.update();

        Logger.recordOutput("AScope/Components", new Pose3d[]{
                new Pose3d(0, 0, climber.getPosition().in(Meters), new Rotation3d()),
                new Pose3d(INTAKE_OFFSET, new Rotation3d(Radians.zero(), intake.getAngle(), Degrees.of(180))),
                new Pose3d(TURRET_OFFSET, new Rotation3d(0, 0, turret.getAngle().in(Radians))),
                new Pose3d(TURRET_OFFSET
                        .plus(HOOD_TO_TURRET_OFFSET.rotateBy(new Rotation3d(0, 0, turret.getAngle().in(Radians)))),
                        new Rotation3d(0,
                                hood.getAngle().minus(Degrees.of(25)).in(Radians),
                                turret.getAngle().in(Radians))),
        });
        Logger.recordOutput("target dist", Meters.of(target.getDistance(drive.getPose().getTranslation())));

        SOTMSolver.getInstance().setLUT(hubTrack ? LaunchLUT.LUTHub : LaunchLUT.LUTPass);
        gauss = rand.nextGaussian(0.4, 0.1);
    }

    public void simPeriodic() {

        SimulatedArena.getInstance().simulationPeriodic();

        Logger.recordOutput("Sim/simulatedVoltage", BatteryVoltageSim.getInstance().calculateVoltage());
        Logger.recordOutput("Sim/FieldSimulation/RobotPose", driveSim.getSimulatedDriveTrainPose());

        Logger.recordOutput("Sim/FieldSimulation/Fuel",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }

    public Command getAutonomousCommand() {
        Command auto;
        if (sysidChooser.get() == null || DriverStation.isFMSAttached()) {
//            auto = autoChooser.getSelected();
            auto = generator.getCommand();
        } else {
            auto = sysidChooser.get();
        }
        return auto;
    }

    public void resetDrivetrain(Pose2d pose) {
        driveSim.setSimulationWorldPose(pose);
    }

    public void enableTeleop() {
        CommandScheduler.getInstance().schedule(drive.teleopDrive());
        // symphony.loadSong("music/the-trout.chrp");
        // symphony.play();
        drive.setCallback();
    }

    public void enableAuto() {

        if (RobotBase.isSimulation()) {
            SimulatedArena.getInstance().resetFieldForAuto();
            //preload 8
            SimFuelManager.getInstance().intake.setGamePiecesCount(8);
        }
    }

    public Drive getDrive() {
        return drive;
    }

    /**
     * get command for turret, shooter, and hood to track current target, lead may be disabled through the useLead var
     *
     * @return
     */
    public Command getTrackCommand() {
        return new HiddenConditionalCommand(
                new ParallelCommandGroup(
                        turret.trackWithLead(() -> hubTrack ? HUB_RADIUS : PASS_RADIUS),
                        launcher.velocityControl(() -> SOTMSolver.getInstance().getParams(false).speed()),
                        hood.positionControl(() -> SOTMSolver.getInstance().getParams(false).hoodAngle()),
                        new InstantCommand(() -> {
                            manual = false;
                        })
                ).withName("track with lead"),

                new ParallelCommandGroup(
                        turret.track(() -> target, () -> hubTrack ? HUB_RADIUS : PASS_RADIUS),
                        launcher.velocityControl(() -> LaunchLUT.get(Meters.of(target.getDistance(drive.getPose().getTranslation())), true, hubTrack ? LaunchLUT.LUTHub : LaunchLUT.LUTPass).speed()),
                        hood.positionControl(() -> LaunchLUT.get(Meters.of(target.getDistance(drive.getPose().getTranslation())), true, hubTrack ? LaunchLUT.LUTHub : LaunchLUT.LUTPass).hoodAngle()),
                        new InstantCommand(() -> {
                            manual = false;
                        })
                ).withName("track without lead"),
                useLead::get).finallyDo(() -> {
            manual = true;
        });
    }

    public Command getManShooterCommand() {
        return new ParallelCommandGroup(
                turret.manPos(() -> Degrees.of(manTurretDegrees.getAsDouble()), false),
                launcher.velocityControl(() -> RPM.of(manShooterRPM.getAsDouble())),
                hood.positionControl(() -> Degrees.of(manHoodDegrees.getAsDouble())),
                new InstantCommand(() -> {
                    manual = true;
                })
        ).finallyDo(() -> {
            manual = false;
        }).withName("manual targeting");
    }

    public Command getSimFireCommand() {
        return new InstantCommand(() -> {
            if (SimFuelManager.getInstance().intake.obtainGamePieceFromIntake()) {
                double launchvel = (launcher.getSpeed().in(RPM)) * 2 * Math.PI * Units.inchesToMeters(2) / 60.0;
                double angle = hood.getAngle().plus(Degrees.of(90)).in(Radians);
                double turretAngle = driveSim.getSimulatedDriveTrainPose().getRotation().getRadians() + turret.getAngle().plus(Degrees.of(180)).in(Radians);
                GamePieceProjectile fuel = new GamePieceProjectile(
                        RebuiltFuelOnField.REBUILT_FUEL_INFO,
                        driveSim.getSimulatedDriveTrainPose().getTranslation().plus(new Translation2d(
                                Math.cos(driveSim.getSimulatedDriveTrainPose().getRotation().getRadians()) * TURRET_OFFSET.getX(),
                                Math.sin(driveSim.getSimulatedDriveTrainPose().getRotation().getRadians()) * TURRET_OFFSET.getX()
                        )),
                        new Translation2d(
                                driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative().vxMetersPerSecond,
                                driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative().vyMetersPerSecond
                        ).plus(new Translation2d(Math.cos(angle) * launchvel,0).rotateBy(new Rotation2d(turretAngle))),
                        Units.inchesToMeters(20),
                        Math.sin(angle) * launchvel,
                        new Rotation3d()
                );

                fuel.withTouchGroundHeight(Inches.of(3).in(Meters));
                fuel.enableBecomesGamePieceOnFieldAfterTouchGround();
                SimulatedArena.getInstance().addGamePieceProjectile(fuel);
            }
        }).andThen(new WaitCommand(gauss)).repeatedly();
    }

    @AutoLogOutput
    private boolean inTrench() {
        Translation2d center = new Translation2d(8.269, 4.038);
        //pos with lead
        Translation2d offsetPos = drive.getPose().getTranslation().plus(
                new Translation2d(//look ahead
                        drive.getFieldChassisSpeeds().vxMetersPerSecond * 0.5,
                        drive.getFieldChassisSpeeds().vyMetersPerSecond * 0.5
                ));
        //pos without lead
        Translation2d pos = drive.getPose().getTranslation();

        double width = 1.75;
        double height = 2;

        double xOffset = 2.61;
        double yOffset = 2.5;

        return
                (//will be in trench
                        (pos.getX() > center.getX() + xOffset && pos.getX() < center.getX() + xOffset + width && pos.getY() > center.getY() + yOffset && pos.getY() < center.getY() + yOffset + height) ||
                                (pos.getX() > center.getX() + xOffset && pos.getX() < center.getX() + xOffset + width && pos.getY() < center.getY() - yOffset && pos.getY() > center.getY() - yOffset - height) ||
                                (pos.getX() < center.getX() - xOffset && pos.getX() > center.getX() - xOffset - width && pos.getY() < center.getY() - yOffset && pos.getY() > center.getY() - yOffset - height) ||
                                (pos.getX() < center.getX() - xOffset && pos.getX() > center.getX() - xOffset - width && pos.getY() > center.getY() + yOffset && pos.getY() < center.getY() + yOffset + height)
                ) ||
                        (//currently in trench
                                (offsetPos.getX() > center.getX() + xOffset && offsetPos.getX() < center.getX() + xOffset + width && offsetPos.getY() > center.getY() + yOffset && offsetPos.getY() < center.getY() + yOffset + height) ||
                                        (offsetPos.getX() > center.getX() + xOffset && offsetPos.getX() < center.getX() + xOffset + width && offsetPos.getY() < center.getY() - yOffset && offsetPos.getY() > center.getY() - yOffset - height) ||
                                        (offsetPos.getX() < center.getX() - xOffset && offsetPos.getX() > center.getX() - xOffset - width && offsetPos.getY() < center.getY() - yOffset && offsetPos.getY() > center.getY() - yOffset - height) ||
                                        (offsetPos.getX() < center.getX() - xOffset && offsetPos.getX() > center.getX() - xOffset - width && offsetPos.getY() > center.getY() + yOffset && offsetPos.getY() < center.getY() + yOffset + height)
                        );
    }

    public Command fire() {
        return Commands.parallel(
                hood.go(),
                new HiddenConditionalCommand(
                        new HiddenConditionalCommand(
                                getSimFireCommand(),
                                Commands.parallel(
                                        kicker.feed(),
                                        indexer.feed()
                                ).withName("shoot"),
                                () -> MODE == RobotMode.SIM
                        ),
                        Commands.run(() -> {}),
                        //block scoring if hub is disabled, still allows passing
                        () -> this.isReady() && (!hubTrack || canScore())
                )
        );
    }
    public FuelVision getFuelVision(){return fuelVision;}

    public Command startLutTimer() {
        return new InstantCommand(() -> {
            currentStartTimestamp = Logger.getTimestamp();
            hoodSetpoint = hood.getSetpoint();
            launcherSetpoint = launcher.getSetpoint();
            distanceToHub = Meters.of(target.getDistance(drive.getPose().getTranslation()));
        });
    }

    public Command saveLutEntry() {
        return new InstantCommand(() -> {
            double deltaTime = Logger.getTimestamp() - currentStartTimestamp;

            String line = "new ShotParams(Meters.of(%f), Degrees.of(%f), RPM.of(%f), Microseconds.of(%f));"
                    .formatted(
                            distanceToHub.in(Meters),
                            hoodSetpoint.in(Degrees),
                            launcherSetpoint.in(RPM),
                            deltaTime
                    );
            Logger.recordOutput("LutEntry", line);
            try {
                Files.writeString(
                        Path.of("/U/lut.txt"),
                        line + "\n",
                        StandardOpenOption.CREATE,
                        StandardOpenOption.APPEND
                );
            }
            catch (Exception e) {
                System.out.println(e.getMessage());
                Alert a = new Alert("saving lut failed", AlertType.kWarning);
                a.set(true);
                // kill ourselves
            }
        });
    }

    @AutoLogOutput
    public boolean isReady() {
        return (readyDebounce.calculate(turret.isReady() && launcher.isReady() && hood.isReady()) || forceFeed.getAsBoolean());
    }

    @AutoLogOutput
    public boolean canScore() {
        return (ShiftTracker.canScore() || !shiftLock.getAsBoolean());
    }

    public Command intake() {
        return intake.intake();
    }
    public Hood getHood(){
        return hood;
    }

    public Command unload(){
        return kicker.reverse();
    }
}
