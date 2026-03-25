package frc.robot.oldautos;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import frc.utils.AllianceUtility;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.utils.ExtraMath;
import frc.utils.LoggedField2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;

/**
 * A {@link edu.wpi.first.wpilibj.smartdashboard.SendableChooser} for selecting an autonomous program.
 * <br/><br/>
 * <p>
 * How to add a new autonomous program.
 *     <ol>
 *         <li>
 *             </li>
 *             <li>
 *                 Add a new method in {@link AutoFactory} that returns a {@link edu.wpi.first.wpilibj2.command.Command}.
 *             </li>
 *             <li>
 *                 Add a new {@link AutoProgram} to {@link AutoChooser#AUTO_PROGRAMS}.
 *             </li>
 *             <li>
 *                 Implement the autonomous program factory method.
 *             </li>
 *             <li>
 *                 Test, test, and test some more.
 *             </li>
 *         </ol>
 * </p>
 * <br/><br/>
 * thanks to 2910 for 3/8 of this code
 */
@SuppressWarnings("unchecked")
public class AutoChooser {

    private final AutoFactory factory;
    private final RobotContainer container;

    private final LoggedField2d field = new LoggedField2d();
    private final LoggedDashboardChooser<AutoProgram> chooser;
    private final LoggedNetworkNumber timeSelector = new LoggedNetworkNumber("SmartDashboard/Auto/Time select", 0.0);

    private static Map<String, Object> groups = null;
    private static final Map<String, StringArraySubscriber> errorSubscribers = new HashMap<>();
    private static final Map<String, StringArraySubscriber> warningSubscribers = new HashMap<>();
    private static final Map<String, StringArraySubscriber> infoSubscribers = new HashMap<>();

    private static String[] errors = {};
    private static String[] warnings = {};
    private static String[] infos = {};

    static {
        try {
            Class<?> sendableAlertsClass = Class.forName("edu.wpi.first.wpilibj.Alert$SendableAlerts");
            Field groupsField = sendableAlertsClass.getDeclaredField("groups");
            groupsField.setAccessible(true);
            groups = (Map<String, Object>) groupsField.get(null);
        } catch (ClassNotFoundException
                 | IllegalArgumentException
                 | IllegalAccessException
                 | NoSuchFieldException
                 | SecurityException e) {
            e.printStackTrace();
        }
    }

    /**
     * Log the current state of all alerts as outputs.
     */
    public static void getAlerts() {
        if (groups == null) return;
        for (String group : groups.keySet()) {

            // Create NetworkTables subscribers
            if (!errorSubscribers.containsKey(group)) {
                errorSubscribers.put(
                        group,
                        NetworkTableInstance.getDefault()
                                .getStringArrayTopic("/SmartDashboard/" + group + "/errors")
                                .subscribe(new String[0]));
            }
            if (!warningSubscribers.containsKey(group)) {
                warningSubscribers.put(
                        group,
                        NetworkTableInstance.getDefault()
                                .getStringArrayTopic("/SmartDashboard/" + group + "/warnings")
                                .subscribe(new String[0]));
            }
            if (!infoSubscribers.containsKey(group)) {
                infoSubscribers.put(
                        group,
                        NetworkTableInstance.getDefault()
                                .getStringArrayTopic("/SmartDashboard/" + group + "/infos")
                                .subscribe(new String[0]));
            }

            // Get values
            errors = errorSubscribers.get(group).get();
            warnings = warningSubscribers.get(group).get();
            infos = infoSubscribers.get(group).get();
        }
    }

    private final List<AutoProgram> AUTO_PROGRAMS = List.of(
//            new AutoProgram("preload", AutoFactory::createPreloadAuto),
//            //depot autos
//            new AutoProgram("trench depot + AI", AutoFactory::createDepotAIAuto),
//            new AutoProgram("bump depot + AI", AutoFactory::createBumpDepotAIAuto),
//            //middle autos
//            new AutoProgram("Left insanity", AutoFactory::createLeftPassAuto),
//            new AutoProgram("Right not very insanity", AutoFactory::createRightPassAuto),
//            //ai only autos
//            new AutoProgram("AI right mid", AutoFactory::createRightAIMidAuto),
//            new AutoProgram("AI left mid", AutoFactory::createLeftAIMidAuto),
//            new AutoProgram("AI right zone", AutoFactory::createRightAIZoneAuto),
//            new AutoProgram("AI left zone", AutoFactory::createLeftAIZoneAuto)
    );


    public AutoChooser(RobotContainer container) {
        chooser = new LoggedDashboardChooser<>("Auto Program");
        chooser.addDefaultOption("none", null);

        for (AutoProgram autoProgram : AUTO_PROGRAMS) {
            chooser.addOption(autoProgram.getLabel(), autoProgram);

        }
        this.factory = new AutoFactory(container);
        this.container = container;

        for (AutoProgram program : AUTO_PROGRAMS) {
            program.update(factory);
        }

        SmartDashboard.putData("Auto/Path", field);
    }

    public Command getSelected() {
        return chooser.get().getCommand(factory);
    }

    public void update() {
            getAlerts();
        if (!DriverStation.isEnabled() && chooser.get() != null) {

            double time = timeSelector.get() * chooser.get().getPathLength(factory);

            field.getObject("trajectory").setPoses(chooser.get().getPoses(factory));
            field.setRobotPose(AllianceUtility.flipPose(container.getDrive().getPose()));
            field.getObject("start pose").setPose(chooser.get().getStartingPose(factory));
            field.getObject("selected pose").setPose(chooser.get().getPoseAtTime(factory, time));

            Logger.recordOutput("Auto/Selected time", ExtraMath.roundToPoint(time, 3));
            Logger.recordOutput("Auto/Total time", (chooser.get().getPathLength(factory)));

            Logger.recordOutput("Auto/Checklist/Robot in position", ExtraMath.poseWithinTolerance(container.getDrive().getPose(), chooser.get().getStartingPose(factory), 0.5, Math.toRadians(20)));
        }
        Logger.recordOutput("Auto/Checklist/Auto selected", chooser.get() != null);
        Logger.recordOutput("Auto/Checklist/FMS connected", DriverStation.isFMSAttached());
        Logger.recordOutput("Auto/Checklist/Joysticks connected", DriverStation.isJoystickConnected(0) && DriverStation.isJoystickConnected(1));
        Logger.recordOutput("Auto/Checklist/No alerts",
                errors.length == 0 &&
                        warnings.length == 0 &&
                        infos.length == 0);
        Logger.recordOutput("Auto/Checklist/Auto set", "Set correct auto program");
        Logger.recordOutput("Auto/Checklist/Odometry correct", "Confirm odometry is same with real position");
        Logger.recordOutput("Auto/Checklist/Controller ports", "Confirm controllers are connected to right ports");
        Logger.recordOutput("Auto/Checklist/DS secure", "Confirm DS is securely attached to shelf");
        // Logger.recordOutput("auto/list/", );
        Logger.recordOutput("Auto/Checklist/Gamepieces loaded", "confirm 8 preload fuel");
    }
}