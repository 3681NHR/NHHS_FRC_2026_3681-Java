package frc.robot.oldautos;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveToFuel;
import frc.robot.constants.DriveConstants;
import frc.utils.AllianceUtility;
import frc.utils.ExtraMath;
import frc.utils.RectZone;

import static edu.wpi.first.units.Units.Radians;

/**
 * A factory for creating autonomous programs
 */
public class AutoFactory {

    private final RobotContainer robotContainer;
    /**
     * Create a new <code>AutoFactory</code>.
     *
     * @param robotContainer The {@link RobotContainer}
     */
    public AutoFactory(final RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    /* Autonomous program factories
     *
     * Factory methods should be added here for each autonomous program.
     * The factory methods must:
     *   1. Be package-private (i.e. no access modifier)
     *   2. Accept no parameters
     *   3. Return a link Command
     */
    private static final Command IDLE_COMMAND = Commands.idle();

    Pair<PathPlannerTrajectory, Command> createIdleAuto() {
        return Pair.of(null, IDLE_COMMAND);
    }

    Pair<PathPlannerTrajectory, Command> createExampleAuto() {
        return Pair.of(
                null,
                Commands.sequence());
    }

    Pair<PathPlannerTrajectory, Command> createRightAIMidAuto() {
        try{

            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("R_to_mid");

            List<PathPlannerTrajectoryState> traj = new LinkedList<>(getTraj(path).getStates());

            
            return Pair.of(
                    new PathPlannerTrajectory(traj),
                    Commands.sequence(
                            robotContainer.getDrive().followPath(path),
                            Commands.parallel(
                                    robotContainer.getTrackCommand(),
                                    robotContainer.fire(),
                                    robotContainer.intake(),
                                    new DriveToFuel(robotContainer.getDrive(), robotContainer.getFuelVision(), () -> AllianceUtility.flipRectZone(new RectZone(5.7, 0.5, 8.3, 7.6)))
                            )));
        } catch (Exception e){
            throw new RuntimeException("Failed to create right AI mid Auto", e);
        }
    }

    Pair<PathPlannerTrajectory, Command> createLeftAIMidAuto() {
        try{
        
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("L_to_mid");

            List<PathPlannerTrajectoryState> traj = new LinkedList<>(getTraj(path).getStates());

            
        return Pair.of(
                new PathPlannerTrajectory(traj),
                Commands.sequence(
                    robotContainer.getDrive().followPath(path),
                Commands.parallel(
                        robotContainer.getTrackCommand(),
                        robotContainer.fire(),
                        robotContainer.intake(),
                        new DriveToFuel(robotContainer.getDrive(), robotContainer.getFuelVision(), () -> AllianceUtility.flipRectZone(new RectZone(5.7, 0.5, 8.3, 7.6)))
                )));
        } catch (Exception e){
            throw new RuntimeException("Failed to create left AI mid Auto", e);
        }
    }

    Pair<PathPlannerTrajectory, Command> createLeftAIZoneAuto() {
        try{
            return Pair.of(
                    null,
                            Commands.parallel(
                                    robotContainer.getTrackCommand(),
                                    robotContainer.fire(),
                                    robotContainer.intake(),
                                    new DriveToFuel(robotContainer.getDrive(), robotContainer.getFuelVision(), () -> AllianceUtility.flipRectZone(new RectZone(0.669,4.8,3.5,7.42))
                            )));
        } catch (Exception e){
            throw new RuntimeException("Failed to create left AI zone Auto", e);
        }
    }

    Pair<PathPlannerTrajectory, Command> createDepotAIAuto() {
        try{

            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("depot");

            List<PathPlannerTrajectoryState> traj = getTraj(path).getStates();

            System.out.println(new PathPlannerTrajectory(traj).getTotalTimeSeconds());
            
            return Pair.of(
                    new PathPlannerTrajectory(traj),
                    Commands.parallel(
                            Commands.sequence(
                                    robotContainer.getDrive().followPath(path),
                                    new DriveToFuel(robotContainer.getDrive(), robotContainer.getFuelVision(), () -> AllianceUtility.flipRectZone(new RectZone(0.669,4.8,3.5,7.42))
                            )),
                            robotContainer.getTrackCommand(),
                            Commands.sequence(
                                    Commands.waitSeconds(3),
                                robotContainer.fire()
                            ),
                            robotContainer.intake()
                            ));
        } catch (Exception e){
            throw new RuntimeException("Failed to create left AI zone Auto", e);
        }
    }

    Pair<PathPlannerTrajectory, Command> createBumpDepotAIAuto() {
        try{

            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("bump_depot");

            List<PathPlannerTrajectoryState> traj = new LinkedList<>(getTraj(path).getStates());

            
            return Pair.of(
                    new PathPlannerTrajectory(traj),
                    Commands.parallel(
                            Commands.sequence(
                                    robotContainer.getDrive().followPath(path),
                                    new DriveToFuel(robotContainer.getDrive(), robotContainer.getFuelVision(), () -> AllianceUtility.flipRectZone(new RectZone(0.669,4.8,3.5,7.42)))
                            ),
                            robotContainer.getTrackCommand(),
                            Commands.sequence(
                                    Commands.waitSeconds(3),
                                    robotContainer.fire()
                            ),
                            robotContainer.intake()
                            ));
        } catch (Exception e){
            throw new RuntimeException("Failed to create left AI zone Auto", e);
        }
    }

    Pair<PathPlannerTrajectory, Command> createRightAIZoneAuto() {
        try{
            return Pair.of(
                    null,
                    Commands.parallel(
                            robotContainer.getTrackCommand(),
                            robotContainer.fire(),
                            robotContainer.intake(),
                            new DriveToFuel(robotContainer.getDrive(), robotContainer.getFuelVision(), () -> AllianceUtility.flipRectZone(new RectZone(0.669,0.65,3.5,2.7))
                            )));
        } catch (Exception e){
            throw new RuntimeException("Failed to create left AI zone Auto", e);
        }
    }

    Pair<PathPlannerTrajectory, Command> createPreloadAuto() {
        return Pair.of(
                null,
                new ParallelCommandGroup(
                        robotContainer.getTrackCommand(),
                        Commands.waitSeconds(1.0).andThen(robotContainer.fire().alongWith(robotContainer.intake()))
                ));
    }
    
    Pair<PathPlannerTrajectory, Command> createExamplePPAuto() {
        try{
            PathPlannerPath path = PathPlannerPath.fromPathFile("m4");

            List<PathPlannerTrajectoryState> traj = new LinkedList<>(getTraj(path).getStates());

            
        return Pair.of(
                new PathPlannerTrajectory(traj),
                Commands.sequence(
                    robotContainer.getDrive().followPath(path)
                ));
        } catch (Exception e){
            throw new RuntimeException("Failed to create Test Auto", e);
        }
    }

    Pair<PathPlannerTrajectory, Command> createDepotAuto() {
        try{
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("depot");

            List<PathPlannerTrajectoryState> traj = new LinkedList<>(getTraj(path).getStates());

            
            return Pair.of(
                    new PathPlannerTrajectory(traj),
                    Commands.parallel(
                            robotContainer.getDrive().followPath(path),
                            robotContainer.getTrackCommand(),
                            Commands.sequence(
                                Commands.waitSeconds(0.8),
                                robotContainer.intake()
                            ),
                            Commands.sequence(
                                    Commands.waitSeconds(0.8),
                                    robotContainer.fire()
                            )
                    ));
        } catch (Exception e){
            throw new RuntimeException("Failed to create Test Auto", e);
        }
    }

    Pair<PathPlannerTrajectory, Command> createLeftPassAuto() {
        try{
            PathPlannerPath swipePath = PathPlannerPath.fromChoreoTrajectory("L_swipe");
            PathPlannerPath transitionPath = PathPlannerPath.fromChoreoTrajectory("L_mid_to_inner_neutral");
            PathPlannerPath depotPath = PathPlannerPath.fromChoreoTrajectory("depot");

            List<PathPlannerTrajectoryState> traj1 = new LinkedList<>(getTraj(swipePath).getStates());
            List<PathPlannerTrajectoryState> traj2 = new LinkedList<>(getTraj(transitionPath).getStates());
            List<PathPlannerTrajectoryState> traj3 = new LinkedList<>(getTraj(depotPath).getStates());

            return Pair.of(
                    mergeTrajectories(new PathPlannerTrajectory(traj1), new PathPlannerTrajectory(traj2), new PathPlannerTrajectory(traj3)),
                    Commands.sequence(
                            new InstantCommand(() -> org.littletonrobotics.junction.Logger.recordOutput("auto stage", 0)),
                            Commands.deadline(//pass
                                robotContainer.getDrive().followPath(swipePath),
                                robotContainer.getTrackCommand(),
                                Commands.sequence(
                                        Commands.waitSeconds(1),//wait until out of trench
                                        Commands.parallel(
                                            robotContainer.intake(),
                                            robotContainer.fire()
                                        )
                                )
                            ),
                            new InstantCommand(() -> org.littletonrobotics.junction.Logger.recordOutput("auto stage", 1)),
                            //lag here
                            robotContainer.getDrive().followPath(transitionPath),
                            new InstantCommand(() -> org.littletonrobotics.junction.Logger.recordOutput("auto stage", 2)),
                            // then here
                            Commands.parallel(//depot
                                    robotContainer.getDrive().followPath(depotPath),
                                    robotContainer.getTrackCommand(),
                                    Commands.sequence(
                                            Commands.waitSeconds(0.8),
                                            robotContainer.intake(),
                                            robotContainer.fire()
                                    )
                            ),
                            new InstantCommand(() -> org.littletonrobotics.junction.Logger.recordOutput("auto stage", 3))
                    ));
        } catch (Exception e){
            throw new RuntimeException("Failed to create Test Auto", e);
        }
    }

    Pair<PathPlannerTrajectory, Command> createRightPassAuto() {
        try{
            PathPlannerPath swipePath = PathPlannerPath.fromChoreoTrajectory("R_swipe");
            PathPlannerPath transitionPath = PathPlannerPath.fromChoreoTrajectory("R_mid_to_zone");

            List<PathPlannerTrajectoryState> traj1 = new LinkedList<>(getTraj(swipePath).getStates());
            List<PathPlannerTrajectoryState> traj2 = new LinkedList<>(getTraj(transitionPath).getStates());

            return Pair.of(
                    mergeTrajectories(new PathPlannerTrajectory(traj1), new PathPlannerTrajectory(traj2)),
                    Commands.sequence(
                            new InstantCommand(() -> org.littletonrobotics.junction.Logger.recordOutput("auto stage", 0)),
                            Commands.deadline(//pass
                                    robotContainer.getDrive().followPath(swipePath),
                                    robotContainer.getTrackCommand(),
                                    Commands.sequence(
                                            Commands.waitSeconds(1),//wait until out of trench
                                            Commands.parallel(
                                                    robotContainer.intake(),
                                                    robotContainer.fire()
                                            )
                                    )
                            ),
                            new InstantCommand(() -> org.littletonrobotics.junction.Logger.recordOutput("auto stage", 1)),
                            //lag here
                            robotContainer.getDrive().followPath(transitionPath),
                            new InstantCommand(() -> org.littletonrobotics.junction.Logger.recordOutput("auto stage", 2)),
                            // then here
                            Commands.parallel(//score
                                    robotContainer.getTrackCommand(),
                                    robotContainer.intake(),
                                    robotContainer.fire()
                            )
                    ));
        } catch (Exception e){
            throw new RuntimeException("Failed to create Test Auto", e);
        }
    }

    @SuppressWarnings("unused")
    private PathPlannerTrajectory mergeTrajectories(PathPlannerTrajectory... in){
        List<PathPlannerTrajectoryState> traj = new LinkedList<>();

        double timeOffset = 0.0;
        for (PathPlannerTrajectory trajectory : in) {
            PathPlannerTrajectoryState[] states = trajectory.getStates().toArray(new PathPlannerTrajectoryState[0]);
            double nextoffset = states[states.length - 1].timeSeconds;
            for (PathPlannerTrajectoryState s : states) {
                s.timeSeconds += timeOffset;
                traj.add(s);
            }
            timeOffset += nextoffset;
        }
        return new PathPlannerTrajectory(traj);
    }
    private PathPlannerTrajectory getTraj(PathPlannerPath path){
        return (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? path.flipPath() : path).getIdealTrajectory(DriveConstants.PP_CONFIG).orElse(null);
    }

    private double getAngleToNearestFuel(){
        Translation2d robotPos = robotContainer.getDrive().getPose().getTranslation();
        ArrayList<Translation2d> fuel = robotContainer.getFuelVision().getTrackedFuel();

        if (!fuel.isEmpty()) {
            return ExtraMath.getAngleToPos(selectFuelTarget(robotPos, fuel), robotPos).in(Radians);
        }
        return robotContainer.getDrive().getPose().getRotation().getRadians();
    }

    private Translation2d selectFuelTarget(Translation2d robotPos, List<Translation2d> candidates) {

            return candidates.stream()
                    .min(Comparator.comparingDouble(e -> e.getDistance(robotPos)))
                    .orElse(robotPos);
    }

}