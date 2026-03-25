package frc.robot.oldautos;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Function;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

/**
 * An autonomous program.
 */
public class AutoProgram {
    private final String label;

    private final Function<AutoFactory, Pair<PathPlannerTrajectory, Command>> commandFactory;

    private Pair<PathPlannerTrajectory, Command> commandPair;
    /**
     * Create an autonomous program
     *
     * @param auto
     * The {@link Auto} to associate with this program
     * @param label
     * The human-readable label for the program
     * @param commandFactory
     * The command factory for the program
     */
    public AutoProgram(final String label, final Function<AutoFactory, Pair<PathPlannerTrajectory, Command>> commandFactory) {
        this.label = label;
        this.commandFactory = commandFactory;

    }
    public void update(AutoFactory factory){
        this.commandPair = commandFactory.apply(factory);
    }

    /**
     * Get the label for this program
     * @return
     * The label for this program
     */
    public String getLabel() {
        return label;
    }

    /**
     * Construct the {@link Command} for this program from the provided {@link AutoFactory}
     *
     * @param autoFactory
     * The {@link AutoFactory} to use when creating the {@link Command}
     * @return
     * The {@link Command} for this program from the provided {@link AutoFactory}
     */
    public Command getCommand(final AutoFactory autoFactory) {
        return commandPair.getSecond();
    }

    public Pose2d getStartingPose(final AutoFactory autoFactory) {
        if(getPoses(autoFactory).length == 0){
            return new Pose2d();
        } else {
            return commandPair.getFirst().getInitialPose();
        }
    }

    public Pose2d[] getPoses(final AutoFactory autoFactory){
        if(commandPair.getFirst() != null){
            return commandPair.getFirst().getStates().stream().map(state -> state.pose).toArray(Pose2d[]::new);
        } else {
            return new Pose2d[0];
        }
    }

    public Pose2d getPoseAtTime(final AutoFactory autoFactory, double time){
        if(commandPair.getFirst() != null){
            return commandPair.getFirst().sample(time).pose;
        } else {
            return new Pose2d();
        }
    }
    
    public double getPathLength(final AutoFactory autoFactory){
        if(commandPair.getFirst() != null){
            return commandPair.getFirst().getTotalTimeSeconds();
        } else {
            return 0.0;
        }
    }
}