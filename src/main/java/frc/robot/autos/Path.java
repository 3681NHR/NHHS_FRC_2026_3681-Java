package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;

import java.util.LinkedList;
import java.util.function.Supplier;

/**
 * enum containing all paths
 * <p>the length of the trajectory should be the same as the execution time of the command for preview accuracy</p>
 */
public enum Path {
    L_TRENCH_TO_DEPOT(
            "Right of depot",
            new Path[]{},//initialized in static block
            getChoreoTraj("A"),
            "path a",
            () -> Commands.parallel(
                    followChoreoPath("A"),
                    Commands.waitSeconds(1).andThen(
                            Commands.parallel(
                                    track(),
                                    intake()
                            )
                    ),
                    Commands.waitSeconds(2.7).andThen(shoot())
            )
    ),
    L_BUMP_TO_DEPOT(
            "Right of depot",
            new Path[]{},//initialized in static block
            addDelayToStart(getChoreoTraj("B"), 1),
            "path b",
            () -> Commands.parallel(
                    followChoreoPath("B"),
                    Commands.waitSeconds(1).andThen(
                            Commands.parallel(
                                    track(),
                                    intake()
                            )
                    ),
                    Commands.waitSeconds(2.7).andThen(shoot())
            )
    ),
    MIDDLE(
            "Middle of zone",
            new Path[]{},//initialized in static block
            getChoreoTraj("C"),
            "path c",
            () -> Commands.sequence(
                    followChoreoPath("C"),
                    Commands.parallel(
                        track(),
                        shoot()
                    )
            )
    ),
    R_BUMP_TO_OUTPOST(
            "Outpost",
            new Path[]{},//initialized in static block
            getChoreoTraj("D"),
            "path d",
            () -> Commands.parallel(
                    followChoreoPath("D"),
                    Commands.waitSeconds(1).andThen(
                            Commands.parallel(
                                    track(),
                                    intake()
                            )
                    ),
                    Commands.waitSeconds(2.5).andThen(shoot())
            )
    ),
    MIDDLE_TO_OUTPOST(
            "Outpost",
            new Path[]{},//initialized in static block
            getChoreoTraj("E"),
            "path e",
            () -> Commands.parallel(
                    followChoreoPath("E"),
                    Commands.waitSeconds(1).andThen(
                            Commands.parallel(
                                    track(),
                                    intake()
                            )
                    ),
                    Commands.waitSeconds(2.6).andThen(shoot())
            )
    ),
    R_TRENCH_TO_OUTPOST(
            "Outpost",
            new Path[]{},//initialized in static block
            getChoreoTraj("F"),
            "path f",
            () -> Commands.parallel(
                    followChoreoPath("F"),
                    Commands.waitSeconds(1).andThen(
                            Commands.parallel(
                                    track(),
                                    intake()
                            )
                    ),
                    Commands.waitSeconds(2.5).andThen(shoot())
            )
    ),
    L_TRENCH_TO_MID_PICKUP(
            "Left side of neutral zone",
            new Path[]{},//initialized in static block
            getChoreoTraj("G"),
            "path g(intake only)",
            () -> Commands.deadline(
                    followChoreoPath("G"),
                    Commands.waitSeconds(1.5).andThen(intake())
            )
    ),
    L_TRENCH_TO_MID_PASS(
            "Left side of neutral zone",
            new Path[]{},//initialized in static block
            getChoreoTraj("G"),
            "path g(pass)",
            () -> Commands.deadline(
                    followChoreoPath("G"),
                    Commands.waitSeconds(1).andThen(track()),
                    Commands.waitSeconds(1.5).andThen(intake()),
                    Commands.waitSeconds(2.7).andThen(shoot())
            )
    ),
    R_TRENCH_TO_MID_PICKUP(
            "Right side of neutral zone",
            new Path[]{},//initialized in static block
            getChoreoTraj("H"),
            "path h(intake only)",
            () -> Commands.deadline(
                    followChoreoPath("H"),
                    Commands.waitSeconds(1.5).andThen(intake())
            )
    ),
    R_TRENCH_TO_MID_PASS(
            "Right side of neutral zone",
            new Path[]{},//initialized in static block
            getChoreoTraj("H"),
            "path h(pass)",
            () -> Commands.deadline(
                    followChoreoPath("H"),
                    Commands.waitSeconds(1).andThen(track()),
                    Commands.waitSeconds(1.5).andThen(intake()),
                    Commands.waitSeconds(2.7).andThen(shoot())
            )
    ),
    L_MID_TO_L_TRENCH(
            "Left trench",
            new Path[]{},//initialized in static block
            getChoreoTraj("I"),
            "path i",
            () -> Commands.deadline(
                    followChoreoPath("I"),
                    lowerHood()
                    )
    ),
    R_MID_TO_R_TRENCH(
            "Right trench",
            new Path[]{},//initialized in static block
            getChoreoTraj("J"),
            "path j",
            () -> Commands.deadline(
                    followChoreoPath("J"),
                    lowerHood()
                    )
    ),
    L_MID_TO_L_BUMP(
            "Left bump",new Path[]{},//initialized in static block
            getChoreoTraj("K"),
            "path k",
            () -> followChoreoPath("K")
    ),
    L_MID_TO_L_BUMP_AND_FIRE(
            "Fire at left bump(5 sec)",new Path[]{},//initialized in static block
            addDelayToEnd(getChoreoTraj("K"), 5),
            "path k(with pause)",
            () -> Commands.deadline(
                    followChoreoPath("K").andThen(Commands.parallel(
                            shoot(),
                            agitate()
                    ).withTimeout(5)),
                    track()
            )
    ),
    R_MID_TO_R_BUMP(
            "Right bump",
            new Path[]{},//initialized in static block//initialized in static block
            getChoreoTraj("L"),
            "path l",
            () -> followChoreoPath("L")
    ),
    R_MID_TO_R_BUMP_AND_FIRE(
            "fire at right bump(5 sec)",
            new Path[]{},//initialized in static block//initialized in static block
            getChoreoTraj("L"),
            "path l(with pause)",
            () -> Commands.deadline(
                    followChoreoPath("L").andThen(Commands.parallel(
                            shoot(),
                            agitate()
                    ).withTimeout(5)),
                    track()
            )
    ),
    START_L_TRENCH("Left trench" , new Path[]{}, null, "", Commands::none),
    START_L_BUMP(  "Left bump"   , new Path[]{},    null,   "", Commands::none),
    START_MID(     "Middle start", new Path[]{}, null,      "", Commands::none),
    START_R_BUMP(  "Right bump"  , new Path[]{},    null,   "", Commands::none),
    START_R_TRENCH("Right trench", new Path[]{}, null, "", Commands::none),
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
            return Commands.none();
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

    private static Command intake(){
        return container.intake();
    }
    private static Command shoot(){
        return container.unload().withTimeout(0.2).andThen(container.fire());
    }
    private static Command track(){
        return container.getTrackCommand();
    }
    private static Command lowerHood(){
        return container.getHood().retract();
    }
    private static Command agitate(){
        return container.intake().withTimeout(1).andThen(Commands.waitSeconds(1)).repeatedly();
    }

    static {
        START_L_TRENCH.options = new Path[]{L_TRENCH_TO_MID_PASS, L_TRENCH_TO_MID_PICKUP,  L_TRENCH_TO_DEPOT};
        START_L_BUMP.options   = new Path[]{L_BUMP_TO_DEPOT};
        START_MID.options      = new Path[]{MIDDLE_TO_OUTPOST, MIDDLE};
        START_R_BUMP.options   = new Path[]{R_BUMP_TO_OUTPOST};
        START_R_TRENCH.options = new Path[]{R_TRENCH_TO_MID_PASS, R_TRENCH_TO_MID_PICKUP,  R_TRENCH_TO_OUTPOST};

        L_TRENCH_TO_DEPOT.options = new Path[]{};
        L_BUMP_TO_DEPOT.options = new Path[]{};
        MIDDLE.options = new Path[]{};
        R_BUMP_TO_OUTPOST.options = new Path[]{};
        MIDDLE_TO_OUTPOST.options = new Path[]{};
        R_TRENCH_TO_OUTPOST.options = new Path[]{};
        L_TRENCH_TO_MID_PASS.options = new Path[]{L_MID_TO_L_TRENCH, L_MID_TO_L_BUMP, L_MID_TO_L_BUMP_AND_FIRE};
        R_TRENCH_TO_MID_PASS.options = new Path[]{R_MID_TO_R_TRENCH, R_MID_TO_R_BUMP, R_MID_TO_R_BUMP_AND_FIRE};

        L_TRENCH_TO_MID_PICKUP.options = L_TRENCH_TO_MID_PASS.options;
        R_TRENCH_TO_MID_PICKUP.options = R_TRENCH_TO_MID_PASS.options;
        L_MID_TO_L_TRENCH.options = START_L_TRENCH.options;
        R_MID_TO_R_TRENCH.options = START_R_TRENCH.options;
        L_MID_TO_L_BUMP_AND_FIRE.options = START_L_BUMP.options;
        L_MID_TO_L_BUMP.options = START_L_BUMP.options;
        R_MID_TO_R_BUMP_AND_FIRE.options = START_R_BUMP.options;
        R_MID_TO_R_BUMP.options = START_R_BUMP.options;

    }
}