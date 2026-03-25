package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.DriveConstants;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;

/**
 * enum containing all paths
 */
public enum Path {
    A(
            "Right of depot",
            new Path[]{},
            Seconds.of(1),
            getChoreoTraj("A"),
            "path a",
            Commands::none
    ),
    B(
            "Right of depot",
            new Path[]{},
            Seconds.of(1),
            getChoreoTraj("B"),
            "path b",
            Commands::none
    ),
    C(
            "Middle of zone",
            new Path[]{},
            Seconds.of(1),
            getChoreoTraj("C"),
            "path c",
            Commands::none
    ),
    D(
            "Outpost",
            new Path[]{},
            Seconds.of(1),
            getChoreoTraj("D"),
            "path d",
            Commands::none
    ),
    E(
            "Outpost",
            new Path[]{},
            Seconds.of(1),
            getChoreoTraj("E"),
            "path e",
            Commands::none
    ),
    F(
            "Outpost",
            new Path[]{},
            Seconds.of(1),
            getChoreoTraj("F"),
            "path f",
            Commands::none
    ),
    G(
            "Left side of neutral zone",
            new Path[]{},
            Seconds.of(1),
            getChoreoTraj("G"),
            "path g",
            Commands::none
    ),
    H(
            "Right side of neutral zone",
            new Path[]{},
            Seconds.of(1),
            getChoreoTraj("H"),
            "path h",
            Commands::none
    ),
    START_L_TRENCH("Left trench" , new Path[]{A, G}, Seconds.zero(), new LinkedList<>(), "", Commands::none),
    START_L_BUMP(  "Left bump"   , new Path[]{B}, Seconds.zero(),    new LinkedList<>(),   "", Commands::none),
    START_MID(     "Middle start", new Path[]{C, E}, Seconds.zero(), new LinkedList<>(),      "", Commands::none),
    START_R_BUMP(  "Right bump"  , new Path[]{D}, Seconds.zero(),    new LinkedList<>(),   "", Commands::none),
    START_R_TRENCH("Right trench", new Path[]{F, H}, Seconds.zero(), new LinkedList<>(), "", Commands::none),
    START("", new Path[]{
            START_L_TRENCH,
            START_L_BUMP,
            START_MID,
            START_R_BUMP,
            START_R_TRENCH
    }, Seconds.zero(), new LinkedList<>(), "", Commands::none),
    ;

    public final String end;
    public final Time length;
    public final List<PathPlannerTrajectoryState> ppPath;
    public final String name;
    public final Path[] options;
    public final Supplier<Command> Command;

    private Path(
            String end,
            Path[] options,
            Time length,
            List<PathPlannerTrajectoryState> ppPath,
            String name,
            Supplier<Command> Command){
        this.options = options;
        this.end = end;
        this.length = length;
        this.ppPath = ppPath;
        this.name = name;
        this.Command = Command;
    }

    private static PathPlannerTrajectory getTraj(PathPlannerPath path){
        return (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? path.flipPath() : path).getIdealTrajectory(DriveConstants.PP_CONFIG).orElse(null);

    }
    private static List<PathPlannerTrajectoryState> getChoreoTraj(String name){
        try {
            return new LinkedList<>(getTraj(PathPlannerPath.fromChoreoTrajectory(name)).getStates());
        } catch (Exception e){
            System.out.println("error getting path: " + name);
            e.printStackTrace();
            return new LinkedList<>();
        }
    }
}