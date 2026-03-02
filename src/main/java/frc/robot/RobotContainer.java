/* I wrote this robot code with furry paws on. Just thought I would mention that. -yarden*/

package frc.robot;

import frc.robot.autos.AutoChooser;
import frc.robot.commands.SwerveWheelCharacterization;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FuelVisionConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.launchLUT;
import frc.robot.subsystems.fuelVision.FuelVision;
import frc.robot.subsystems.fuelVision.FuelVisionIOPhoton;
import frc.robot.subsystems.fuelVision.FuelVisionIOPhotonSim;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherIO;
import frc.robot.subsystems.launcher.LauncherIOReal;
import frc.robot.subsystems.launcher.LauncherIOSim;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOCrackingSpark;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOMini;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOPhoton;
import frc.robot.subsystems.vision.CameraIOPhotonSim;
import frc.robot.subsystems.vision.Vision;
import frc.utils.rumble.*;
import frc.utils.Joystick.duelJoystickAxis;
import frc.utils.TimerHandler;
import frc.utils.BatteryVoltageSim;
import frc.utils.ExtraMath;
import frc.utils.Joystick;
import frc.utils.PIDTuner;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.constants.TurretConstants.*;

import static frc.utils.ControllerMap.*;

import java.util.Arrays;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


public class RobotContainer {
    private LoggedDashboardChooser<Command> sysidChooser = new LoggedDashboardChooser<Command>("sysid auto chooser");

    private DriveTrainSimulationConfig driveTrainSimulationConfig;
    private SwerveDriveSimulation driveSim;
    private Drive drive;
    private Vision vision;
    private FuelVision fuelVision;
    private Turret turret;
    private Launcher launcher;

    private Led led = new Led();

    private final XboxController driverController = new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final XboxController operatorController = new XboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    private LoggedNetworkBoolean resetOdometry = new LoggedNetworkBoolean("Debug/Reset Odometry", false);
    private AutoChooser autoChooser;

    private RumbleHandler rumbler = new RumbleHandler(driverController);
    private RumbleHandler opRumbler = new RumbleHandler(operatorController);

    private PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);

    private Translation2d target = new Translation2d();

    AprilTagFieldLayout apriltagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
    private final Alert operatorDisconnected = new Alert("Operator controller disconnected (port 1).",
            AlertType.kWarning);

    private duelJoystickAxis driverSticks;

    public RobotContainer() {

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
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
        //     driveSim.
        }

        // process driver controls(radial deadzone, curve, trigger slowdown, and
        // inversion)
        driverSticks = new duelJoystickAxis(
                () -> ExtraMath.processInput(
                        Joystick.deadzone(Constants.OperatorConstants.LEFT_DEADBAND,
                                driverController.getRawAxis(LEFT_STICK_X), driverController.getRawAxis(LEFT_STICK_Y))
                                .getX(),
                        -1.0,
                        Constants.OperatorConstants.TRANSLATION_CURVE, 0.0),
                () -> ExtraMath.processInput(
                        Joystick.deadzone(Constants.OperatorConstants.LEFT_DEADBAND,
                                driverController.getRawAxis(LEFT_STICK_X), driverController.getRawAxis(LEFT_STICK_Y))
                                .getY(),
                        -1.0,
                        Constants.OperatorConstants.TRANSLATION_CURVE, 0.0),
                () -> ExtraMath.processInput(
                        Joystick.deadzone(Constants.OperatorConstants.RIGHT_DEADBAND,
                                driverController.getRawAxis(RIGHT_STICK_X), driverController.getRawAxis(RIGHT_STICK_Y))
                                .getX(),
                        -0.75,
                        Constants.OperatorConstants.ROTATION_CURVE, 0.0),
                () -> ExtraMath.processInput(
                        Joystick.deadzone(Constants.OperatorConstants.RIGHT_DEADBAND,
                                driverController.getRawAxis(RIGHT_STICK_X), driverController.getRawAxis(RIGHT_STICK_Y))
                                .getY(),
                        -1.0,
                        Constants.OperatorConstants.ROTATION_CURVE, 0.0));

        switch (Constants.MODE) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                vision = new Vision(
                        apriltagLayout,
                        new CameraIOPhoton(apriltagLayout, VisionConstants.CAMERA_CONFIGS[0]),
                        new CameraIOPhoton(apriltagLayout, VisionConstants.CAMERA_CONFIGS[1]),
                        new CameraIOPhoton(apriltagLayout, VisionConstants.CAMERA_CONFIGS[2]),
                        new CameraIOPhoton(apriltagLayout, VisionConstants.CAMERA_CONFIGS[3])
                        );
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOCrackingSpark(0),
                        new ModuleIOCrackingSpark(1),
                        new ModuleIOCrackingSpark(2),
                        new ModuleIOCrackingSpark(3),
                        vision,
                        driverSticks,
                        led);
                fuelVision = new FuelVision(new FuelVisionIOPhoton(FuelVisionConstants.CAMERA_CONFIG), drive::getPose);
                
                turret = new Turret(new TurretIOReal(), drive);
                launcher = new Launcher(new LauncherIOReal());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                vision = new Vision(
                        apriltagLayout,
                        new CameraIOPhotonSim(apriltagLayout, VisionConstants.CAMERA_CONFIGS[0], driveSim::getSimulatedDriveTrainPose),
                        new CameraIOPhotonSim(apriltagLayout, VisionConstants.CAMERA_CONFIGS[1], driveSim::getSimulatedDriveTrainPose),
                        new CameraIOPhotonSim(apriltagLayout, VisionConstants.CAMERA_CONFIGS[2], driveSim::getSimulatedDriveTrainPose),
                        new CameraIOPhotonSim(apriltagLayout, VisionConstants.CAMERA_CONFIGS[3], driveSim::getSimulatedDriveTrainPose));
                if (driveSim != null) {
                    drive = new Drive(
                            new GyroIOSim(driveSim.getGyroSimulation()) {
                            },
                            new ModuleIOSim(driveSim.getModules()[0]),
                            new ModuleIOSim(driveSim.getModules()[1]),
                            new ModuleIOSim(driveSim.getModules()[2]),
                            new ModuleIOSim(driveSim.getModules()[3]),
                            vision,
                            driverSticks,
                            led);
                fuelVision = new FuelVision(new FuelVisionIOPhotonSim(FuelVisionConstants.CAMERA_CONFIG, driveSim::getSimulatedDriveTrainPose), drive::getPose);
                turret = new Turret(new TurretIOSim(), drive);
                launcher = new Launcher(new LauncherIOSim());
                }
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
                        driverSticks,
                        led);
                turret = new Turret(new TurretIO() {}, drive);
                launcher = new Launcher(new LauncherIO() {});
                break;
        }

        // build pathplanner autos and put in dashboard
        // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        configureBindings();

        autoChooser = new AutoChooser(this);
        sysidChooser.addDefaultOption("none", null);
        sysidChooser.addOption("drive sysid quasistatic forward", drive.sysIdQuasistatic(Direction.kForward));
        sysidChooser.addOption("drive sysid quasistatic reverse", drive.sysIdQuasistatic(Direction.kReverse));
        sysidChooser.addOption("drive sysid dynamic forward",     drive.sysIdDynamic(Direction.kForward));
        sysidChooser.addOption("drive sysid dynamic reverse",     drive.sysIdDynamic(Direction.kReverse));
        sysidChooser.addOption("steer sysid quasistatic forward", drive.steerSysIdQuasistatic(Direction.kForward));
        sysidChooser.addOption("steer sysid quasistatic reverse", drive.steerSysIdQuasistatic(Direction.kReverse));
        sysidChooser.addOption("steer sysid dynamic forward",     drive.steerSysIdDynamic(Direction.kForward));
        sysidChooser.addOption("steer sysid dynamic reverse",     drive.steerSysIdDynamic(Direction.kReverse));
        sysidChooser.addOption("angle sysid quasistatic forward", drive.angleSysIdQuasistatic(Direction.kForward));
        sysidChooser.addOption("angle sysid quasistatic reverse", drive.angleSysIdQuasistatic(Direction.kReverse));
        sysidChooser.addOption("angle sysid dynamic forward",     drive.angleSysIdDynamic(Direction.kForward));
        sysidChooser.addOption("angle sysid dynamic reverse",     drive.angleSysIdDynamic(Direction.kReverse));
        sysidChooser.addOption("swerve wheel radius char",     new SwerveWheelCharacterization(drive));
        sysidChooser.addOption("turret sysid quasistatic forward", turret.sysidQuasistatic(false));
        sysidChooser.addOption("turret sysid quasistatic reverse", turret.sysidQuasistatic(true));
        sysidChooser.addOption("turret sysid dynamic forward",     turret.sysidDynamic(false));
        sysidChooser.addOption("turret sysid dynamic reverse",     turret.sysidDynamic(true));
        sysidChooser.addOption("launcher sysid quasistatic forward", launcher.sysidQuasistatic(false));
        sysidChooser.addOption("launcher sysid quasistatic reverse", launcher.sysidQuasistatic(true));
        sysidChooser.addOption("launcher sysid dynamic forward",     launcher.sysidDynamic(false));
        sysidChooser.addOption("launcher sysid dynamic reverse",     launcher.sysidDynamic(true));

        LoggedPowerDistribution.getInstance(pdp.getModule(), ModuleType.kRev);

        turret.setDefaultCommand(
            turret.manPos(turret::getAngle).ignoringDisable(true)
        );

        launcher.setDefaultCommand(
            launcher.velocityControl(() -> RPM.of(0)).ignoringDisable(true)
        );

        drive.setDefaultCommand(drive.TeleopDrive());
    }

    private void configureBindings() {
        // new Trigger(DriverStation::isDisabled).onTrue();

        // reset odometry dashboard button
        resetOdometry.set(false);
        new Trigger(() -> resetOdometry.get()).onTrue(new InstantCommand(() -> {
            resetOdometry.set(false);
            drive.setPose(Constants.STARTING_POSE);
        }));

        // send haptic command when 25 seconds are left in teleops
        new Trigger(() -> TimerHandler.getTeleopRemaining() < 25.0).onTrue(new InstantCommand(() -> {
            rumbler.overrideQue(RumblePreset.TAP.load());
            opRumbler.overrideQue(RumblePreset.TAP.load());
        }));

        // ------------------------------------------------------------------------------
        // driver controls
        // ------------------------------------------------------------------------------

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
        
        // TODO: placeholder binding to shooting in sim, remove before running on robot
        // new Trigger(() -> driverController.getRawAxis(RIGHT_TRIGGER) > 0.7).whileTrue(new InstantCommand(() -> {
        //         double launchvel = (launcher.getSpeed().in(RPM)-2500)*2*Math.PI*Units.inchesToMeters(2)/60;
        //         double angle = launchLUT.get(target.getDistance(turret.getFieldPos()), true, launchLUT.LUTHub)[0];
        //         GamePieceProjectile fuel = new GamePieceProjectile(
        //                 RebuiltFuelOnField.REBUILT_FUEL_INFO,
        //                 driveSim.getSimulatedDriveTrainPose().getTranslation().plus(new Translation2d(
        //                         Math.cos(drive.getRotation().getRadians())*TURRET_OFFSET.getX(),
        //                         Math.sin(drive.getRotation().getRadians())*TURRET_OFFSET.getX()
        //                 )),
        //                 new Translation2d(
        //                         ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation()).vxMetersPerSecond + Math.cos(drive.getRotation().getRadians() + turret.getAngle().in(Radians))*Math.cos(angle)*launchvel,
        //                         ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation()).vyMetersPerSecond + Math.sin(drive.getRotation().getRadians() + turret.getAngle().in(Radians))*Math.cos(angle)*launchvel
        //                 ),
        //                 Units.inchesToMeters(20),
        //                 Math.sin(angle)*launchvel,
        //                 new Rotation3d()
        //                 );
                
        //         fuel.withTouchGroundHeight(Inches.of(3).in(Meters));
        //         fuel.enableBecomesGamePieceOnFieldAfterTouchGround();
        //         SimulatedArena.getInstance().addGamePieceProjectile(fuel);
        // }).andThen(new WaitCommand(0.1)).repeatedly());

        // force teleop drive
        new Trigger(() -> driverController.getPOV() == 0).onTrue(drive.TeleopDrive());

        new Trigger(() -> driverController.getRawButton(X)).onTrue(drive.rotationLock(() -> 0));

        //launcher spin and shoot
        new Trigger(() -> driverController.getRawAxis(RIGHT_TRIGGER) > 0.2).whileTrue(
            launcher.velocityControl(() -> RPM.of(launchLUT.get(target.getDistance(turret.getFieldPos()), true, launchLUT.LUTHub)[1]))
        );
        // new Trigger(() -> driverController.getRawAxis(RIGHT_TRIGGER) > 0.7).whileTrue(
        //     null// TODO: feed to shooter while spun up
        // );
        //set turret to auto track mode
        new Trigger(() -> driverController.getRawButton(B)).onTrue(
            turret.track(() -> target)
        );
        //set turret to preset angle mode
        new Trigger(() -> driverController.getRawButton(A)).onTrue(
            turret.manPos(() -> Degrees.of(0))
        );
        //intake
        // new Trigger(() -> driverController.getRawAxis(LEFT_TRIGGER) > 0.5).whileTrue(
        //     null// TODO: intake
        // );
        // //lower hood for trench(should be auto also)(hold)
        // new Trigger(() -> driverController.getRawButton(X)).whileTrue(
        //     null // TODO: lower hood for trench(should be auto also)(hold)
        // );

        // ------------------------------------------------------------------------------
        // operator controls
        // ------------------------------------------------------------------------------

    }

    public void Periodic() {
        rumbler.update(0.02);
        PIDTuner.updateTunables();
        driverDisconnected.set(!driverController.isConnected());
        operatorDisconnected.set(!operatorController.isConnected());
        
        Translation2d hub = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? TurretConstants.RED_HUB : TurretConstants.BLUE_HUB;
        Translation2d pass = drive.getPose().getTranslation().nearest(Arrays.asList(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? RED_PASS : BLUE_PASS));
        boolean hubTrack = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? drive.getPose().getX()>12 : drive.getPose().getX()<4.5);
        Logger.recordOutput("Subsystems/Turret/track/tracking hub", hubTrack);
        target = hubTrack ? hub : pass;
    
        autoChooser.update();

        Logger.recordOutput("AScope/Components", new Pose3d[]{
                new Pose3d(TURRET_OFFSET, new Rotation3d(0,0,turret.getAngle().in(Radians)-Math.PI/2)),
                new Pose3d(TURRET_OFFSET.getX()+Math.cos(turret.getAngle().in(Radians))*HOOD_TO_TURRET_OFFSET.getX(),TURRET_OFFSET.getY()+(Math.sin(turret.getAngle().in(Radians))*HOOD_TO_TURRET_OFFSET.getX()), TURRET_OFFSET.getZ()+HOOD_TO_TURRET_OFFSET.getZ(),new Rotation3d(Units.degreesToRadians(0),0,turret.getAngle().in(Radians)-Math.PI/2)),
        });
    }

    public void SimPeriodic() {

        SimulatedArena.getInstance().simulationPeriodic();

        Logger.recordOutput("Sim/simulatedVoltage", BatteryVoltageSim.getInstance().calculateVoltage());
        Logger.recordOutput("Sim/FieldSimulation/RobotPose", driveSim.getSimulatedDriveTrainPose());

        Logger.recordOutput("Sim/FieldSimulation/Fuel",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }

    public Command getAutonomousCommand() {
        Command auto;
        if(sysidChooser.get() == null || DriverStation.isFMSAttached()){
            auto = autoChooser.getSelected();
        } else {
            auto = sysidChooser.get();
        }
        return auto;
    }

    public void resetDrivetrain(Pose2d pose) {
        driveSim.setSimulationWorldPose(pose);
    }

    public void enableTeleop() {
        CommandScheduler.getInstance().schedule(drive.TeleopDrive());
        // symphony.loadSong("music/the-trout.chrp");
        // symphony.play();
        drive.setCallback();
    }

    public void enableAuto() {
    }

    public Drive getDrive() {
        return drive;
    }
}
