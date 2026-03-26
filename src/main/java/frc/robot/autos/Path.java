package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;

/**
 * enum containing all paths
 * <p>the length of the trajectory should be the same as the execution time of the command for preview accuracy</p>
 */
public enum Path {
    A(
            "Right of depot",
            new Path[]{},//initialized in static block
            getChoreoTraj("A"),
            "path a",
            () -> followChoreoPath("A")
    ),
    B(
            "Right of depot",
            new Path[]{},//initialized in static block
            addDelayToStart(getChoreoTraj("B"), 1),
            "path b",
            () -> followChoreoPath("B")
    ),
    C(
            "Middle of zone",
            new Path[]{},//initialized in static block
            getChoreoTraj("C"),
            "path c",
            () -> followChoreoPath("C")
    ),
    D(
            "Outpost",
            new Path[]{},//initialized in static block
            getChoreoTraj("D"),
            "path d",
            () -> followChoreoPath("D")
    ),
    E(
            "Outpost",
            new Path[]{},//initialized in static block
            getChoreoTraj("E"),
            "path e",
            () -> followChoreoPath("E")
    ),
    F(
            "Outpost",
            new Path[]{},//initialized in static block
            getChoreoTraj("F"),
            "path f",
            () -> followChoreoPath("F")
    ),
    G(
            "Left side of neutral zone",
            new Path[]{},//initialized in static block
            getChoreoTraj("G"),
            "path g",
            () -> followChoreoPath("G")
    ),
    H(
            "Right side of neutral zone",
            new Path[]{},//initialized in static block
            getChoreoTraj("H"),
            "path h",
            () -> followChoreoPath("H")
    ),
    I(
            "Left trench",
            new Path[]{},//initialized in static block
            getChoreoTraj("I"),
            "path i",
            () -> followChoreoPath("I")
    ),
    J(
            "Right trench",
            new Path[]{},//initialized in static block
            getChoreoTraj("J"),
            "path j",
            () -> followChoreoPath("J")
    ),
    K(
            "Left bump",new Path[]{},//initialized in static block
            addDelayToEnd(getChoreoTraj("K"), 1),
            "path k",
            () -> followChoreoPath("K")
    ),
    L(
            "Right bump",
            new Path[]{},//initialized in static block//initialized in static block
            getChoreoTraj("L"),
            "path l",
            () -> followChoreoPath("L")
    ),
    START_L_TRENCH("Left trench" , new Path[]{A, G}, null, "", Commands::none),
    START_L_BUMP(  "Left bump"   , new Path[]{B},    null,   "", Commands::none),
    START_MID(     "Middle start", new Path[]{C, E}, null,      "", Commands::none),
    START_R_BUMP(  "Right bump"  , new Path[]{D},    null,   "", Commands::none),
    START_R_TRENCH("Right trench", new Path[]{F, H}, null, "", Commands::none),
    START("", new Path[]{
            START_L_TRENCH,
            START_L_BUMP,
            START_MID,
            START_R_BUMP,
            START_R_TRENCH
    }, new LinkedList<>(), "", Commands::none),
    ;
    public static RobotContainer container;

    public final String end;
    public final LinkedList<PathPlannerTrajectoryState> ppPath;
    public final String name;
    public Path[] options;
    public final Supplier<Command> Command;

    private Path(
            String end,
            Path[] options,
            LinkedList<PathPlannerTrajectoryState> ppPath,
            String name,
            Supplier<Command> Command){
        this.options = options;
        this.end = end;
        this.ppPath = ppPath;
        this.name = name;
        this.Command = Command;
    }

    private static PathPlannerTrajectory getTraj(PathPlannerPath path){
        return (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? path.flipPath() : path).getIdealTrajectory(DriveConstants.PP_CONFIG).orElse(null);

    }
    private static LinkedList<PathPlannerTrajectoryState> getChoreoTraj(String name){
        try {
            return new LinkedList<>(getTraj(PathPlannerPath.fromChoreoTrajectory(name)).getStates());
        } catch (Exception e){
            System.out.println("error getting path: " + name);
            e.printStackTrace();
            return new LinkedList<>();
        }
    }
    private static Command followChoreoPath(String pathName) {
        try {
            return container.getDrive().followPath(PathPlannerPath.fromChoreoTrajectory(pathName));
        } catch (Exception e){
            e.printStackTrace();
            return null;
        }
    }

    private static LinkedList<PathPlannerTrajectoryState> addDelayToEnd(LinkedList<PathPlannerTrajectoryState> in, double delay){
        in.add(in.getLast().copyWithTime(in.getLast().timeSeconds+delay));
        return in;
    }

    private static LinkedList<PathPlannerTrajectoryState> addDelayToStart(LinkedList<PathPlannerTrajectoryState> in, double delay){
        for(var v : in){
            v.timeSeconds += delay;
        }
        return in;
    }

    private LinkedList<PathPlannerTrajectoryState> mergeTrajectories(LinkedList<PathPlannerTrajectoryState>... in){
        LinkedList<PathPlannerTrajectoryState> traj = new LinkedList<>();

        double timeOffset = 0.0;
        for (LinkedList<PathPlannerTrajectoryState> trajectory : in) {
            PathPlannerTrajectoryState[] states = trajectory.toArray(new PathPlannerTrajectoryState[0]);
            double nextoffset = (states.length>0 ? states[states.length - 1].timeSeconds : 0.0);
            for (PathPlannerTrajectoryState s : states) {
                traj.add(s.copyWithTime(s.timeSeconds + timeOffset));
            }
            timeOffset += nextoffset;
        }
        return traj;
    }

    static {
        A.options = new Path[]{};
        B.options = new Path[]{};
        C.options = new Path[]{};
        D.options = new Path[]{};
        E.options = new Path[]{};
        F.options = new Path[]{};
        G.options = new Path[]{I, K};
        H.options = new Path[]{J, L};
        I.options = new Path[]{G, A};
        J.options = new Path[]{H, F};
        K.options = new Path[]{B};
        L.options = new Path[]{D};
    }
}