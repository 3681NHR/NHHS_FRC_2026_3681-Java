package frc.robot.autos;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.utils.AllianceUtility;
import frc.utils.LoggedField2d;
import frc.utils.Periodic;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Objects;

public class AutoGenerator extends Periodic {

    LinkedList<Path> sequence = new LinkedList<>();
    LoggedDashboardChooser<Path> chooser = new LoggedDashboardChooser<>("auto/selector");
    LoggedNetworkBoolean enter = new LoggedNetworkBoolean("SmartDashboard/auto/enter", false);
    LoggedNetworkBoolean back = new LoggedNetworkBoolean("SmartDashboard/auto/back", false);
    LoggedNetworkNumber timeSelector = new LoggedNetworkNumber("SmartDashboard/auto/play bar", 0);
    LoggedNetworkBoolean animate = new LoggedNetworkBoolean("SmartDashboard/auto/play\\pause", false);
    LoggedField2d field = new LoggedField2d();
    RobotContainer container;

    public AutoGenerator(RobotContainer container){
        this.container = container;
        populateChooser(Path.START, true);
        generateText();

        SmartDashboard.putData("auto/map", field);
    }

    @Override
    public void update(){
        if(animate.get()){
            timeSelector.set(MathUtil.inputModulus(timeSelector.get()+(0.02/getTotalTime()), 0, 1));
        }
        if(enter.getAsBoolean() && chooser.get() != null){
            enter(chooser.get());
            enter.set(false);
        }
        if(back.getAsBoolean()){

            if(!sequence.isEmpty())
                sequence.removeLast();
            if(!sequence.isEmpty()){
                populateChooser(sequence.getLast(), false);
            } else {
                populateChooser(Path.START, true);
            }
            generateText();

            back.set(false);
        }
        field.getObject("traj").setPoses(
                sequence.stream()
                        .map(path -> path.ppPath)
                        .filter(traj -> traj != null && !traj.isEmpty())
                        .flatMap(List::stream)
                        .map(state -> state.pose)
                        .toArray(Pose2d[]::new));
        field.setRobotPose(getPoseAtTime(timeSelector.get()*getTotalTime()));
        if(!sequence.isEmpty() && sequence.getLast().ppPath != null)
            field.getObject("endPose").setPose(sequence.getLast().ppPath.getLast().pose);
        field.getObject("currPose").setPose(AllianceUtility.flipPose(container.getDrive().getPose().rotateAround(AllianceUtility.FIELD_CENTER_POINT.getTranslation(), Rotation2d.k180deg)));
        Logger.recordOutput("auto/selected time", timeSelector.get()*getTotalTime());
    }
    private void enter(Path e){
        if(chooser.get() != null) {
            sequence.add(chooser.get());
            populateChooser(chooser.get(), false);
            generateText();
        }
    }

    private void populateChooser(Path point, boolean startingPoint){
        chooser.getSendableChooser().close();
        chooser = new LoggedDashboardChooser<>("auto/selector");
//        chooser.onChange(this::enter);
        for(Path p : point.options){
            if(startingPoint){
                chooser.addOption(String.format("start at %s", p.end), p);
            }else {
                chooser.addOption(String.format("%s via %s", p.end, p.name), p);
            }

        }
    }

    private void generateText() {
        if(!sequence.isEmpty()){

            StringBuilder out = new StringBuilder(sequence.getFirst().end);
            if (sequence.size() > 1) {
                ListIterator<Path> i = sequence.listIterator(1);
                while (i.hasNext()) {
                    Path p = i.next();
                    out.append(String.format(" =>(%s) %s", p.name, p.end));
                }
            }
            Logger.recordOutput("auto/generated", out.toString());
        } else {
            Logger.recordOutput("auto/generated", "select a starting point first!");
        }
    }

    @AutoLogOutput(key="auto/estimated time")
    private double getTotalTime(){
        if(sequence.size() <= 1){
            return 0;
        }
        return mergeTrajectories(sequence.stream()
                .map(path -> path.ppPath)
                .filter( traj -> traj != null && !traj.isEmpty())
                .map(PathPlannerTrajectory::new)
                .toArray(PathPlannerTrajectory[]::new)).getTotalTimeSeconds();
    }

    private Pose2d getPoseAtTime(double time){
        if(sequence.size() <= 1){
            if(sequence.isEmpty()){
                return new Pose2d();
            }
            //NOTE: this assumes the starting points all have at least 1 possible next path
            return sequence.getFirst().options[0].ppPath.getFirst().pose;
        }
        return mergeTrajectories(sequence.stream()
                .map(path -> path.ppPath)
                .filter(traj -> traj != null && !traj.isEmpty())
                .map(PathPlannerTrajectory::new)
                .toArray(PathPlannerTrajectory[]::new)).sample(time).pose;
    }


    private PathPlannerTrajectory mergeTrajectories(PathPlannerTrajectory... in){
        List<PathPlannerTrajectoryState> traj = new LinkedList<>();

        double timeOffset = 0.0;
        for (PathPlannerTrajectory trajectory : in) {
            PathPlannerTrajectoryState[] states = trajectory.getStates().toArray(new PathPlannerTrajectoryState[0]);
            double nextoffset = (states.length>0 ? states[states.length - 1].timeSeconds : 0.0);
            for (PathPlannerTrajectoryState s : states) {
//                s.timeSeconds += timeOffset;
                traj.add(s.copyWithTime(s.timeSeconds + timeOffset));
            }
            timeOffset += nextoffset;
        }
        return new PathPlannerTrajectory(traj);
    }

    public Command getCommand(){
        return Commands.sequence(sequence.stream().map(path -> path.Command.get()).toArray(Command[]::new));
    }
}
