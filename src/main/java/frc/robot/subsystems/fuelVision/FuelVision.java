package frc.robot.subsystems.fuelVision;

import java.util.*;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import frc.utils.ExtraMath;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.FuelVisionConstants.*;

public class FuelVision extends SubsystemBase {
    private final FuelVisionIO io;
    private final FuelVisionIOInputsAutoLogged inputs = new FuelVisionIOInputsAutoLogged();
    private final Supplier<Pose2d> pose;

    private final ArrayList<fuelData> newFuel = new ArrayList<>();
    private final Map<gridCoord, Set<fuelData>> fuelMap = new HashMap<>();

    private List<Pair<Double, Pose2d>> driveHistory = new ArrayList<>(10);
    ArrayList<Translation3d> cellsToRender = new ArrayList<>();

    public FuelVision(FuelVisionIO io, Supplier<Pose2d> pose){
        this.io = io;
        this.pose = pose;
    }

    @Override
    public void periodic(){
        inputs.timestamp = Double.NaN;
        io.updateInputs(inputs);
        Logger.processInputs("IO/FuelVision", inputs);


        while(driveHistory.size() > 10){
            driveHistory.remove(0);
        }

        driveHistory.add(new Pair<>(Microseconds.of(Logger.getTimestamp()).in(Seconds), pose.get()));

        //only update if new data
        if(Double.isFinite(inputs.timestamp)) {
            newFuel.clear();

            int i = 0;
            
            while(i < driveHistory.size()-1 && driveHistory.get(i+1).getFirst() < inputs.timestamp){
                i++;
            }
            List<Pair<Double, Pose2d>> getI = driveHistory.get(i);
            Pose2d drivePos;
            if(i+1 < driveHistory.size()) {
                
                double t = (inputs.timestamp - getI.getFirst()) / ((getI+1).getFirst() - getI.getFirst());

                drivePos = new Pose2d(
                        ExtraMath.lerp(getI.getSecond().getX(), (getI+1).getSecond().getX(), t),
                        ExtraMath.lerp(getI.getSecond().getY(), (getI+1).getSecond().getY(), t),
                        new Rotation2d(ExtraMath.lerp(getI.getSecond().getRotation().getRadians(), (getI+1).getSecond().getRotation().getRadians(), t))
                );
                Logger.recordOutput("Subsystems/Fuel Vision/debug/i+1 time", driveHistory.get(i+1).getFirst());
                Logger.recordOutput("Subsystems/Fuel Vision/debug/lerp time", ExtraMath.lerp(getI.getFirst(), (getI+1).getFirst(), t));
            } else {
                drivePos = getI.getSecond();
            }
            Logger.recordOutput("Subsystems/Fuel Vision/debug/frame time", inputs.timestamp);
            Logger.recordOutput("Subsystems/Fuel Vision/debug/i time", getI.getFirst());
            Logger.recordOutput("Subsystems/Fuel Vision/debug/framePos", drivePos);

            for (FuelObservation o : inputs.observations) {
                double d = (CAMERA_CONFIG.robotToCam.getZ() - FUEL_RADIUS.in(Meters)) /
                        Math.tan(CAMERA_CONFIG.robotToCam.getRotation().getY() - o.screenPos().y())
                        / Math.cos(o.screenPos().x());

                //ignore fuel out of bounds
                if (d > MAX_DETECTION_DIST.in(Meters) || d < MIN_DETECTION_DIST.in(Meters)) {
                    continue;
                }

                newFuel.add(new fuelData(toFieldRelative(new Translation2d(
                        d * Math.cos(-CAMERA_CONFIG.robotToCam.getRotation().getX() - o.screenPos().x()),
                        d * Math.sin(-CAMERA_CONFIG.robotToCam.getRotation().getX() - o.screenPos().x())), drivePos),
                        (long) (inputs.timestamp) * 1000000));
            }

            clearFuelInFov(drivePos);
            clearFuelInRobot(drivePos);

            newFuel.forEach(this::addToMap);

        }
        updateFuelPermanence();

        deduplicateFuelMap();

        Logger.recordOutput("Subsystems/Fuel Vision/current detected fuel", newFuel.stream().map(e -> new Translation3d(e.pos.getX(), e.pos.getY(), FUEL_RADIUS.in(Meters))).toArray(Translation3d[]::new));
        Logger.recordOutput("Subsystems/Fuel Vision/all mapped fuel", getTrackedFuel().stream()
                .map(e -> new Translation3d(e.getX(), e.getY(), FUEL_RADIUS.in(Meters)))
                .toArray(Translation3d[]::new));

        //trajectory to render fov in ascope
        {
            Logger.recordOutput("Subsystems/Fuel Vision/fov", new Translation2d[]{

                    new Translation2d(MIN_CLEAR_DETECTION_DIST.in(Meters), 0)//one edge of fov
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 2)))
                            .plus(pose.get().getTranslation()),

                    new Translation2d(MAX_CLEAR_DETECTION_DIST.in(Meters), 0)//one edge of fov
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 2)))
                            .plus(pose.get().getTranslation()),

                    new Translation2d(MAX_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 2.5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 3)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 3.5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 4)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 6)))
                            .plus(pose.get().getTranslation()),
                    //center
                    new Translation2d(MAX_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation())
                            .plus(pose.get().getTranslation()),


                    new Translation2d(MAX_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 6)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 4)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 3.5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 3)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MAX_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 2.5)))
                            .plus(pose.get().getTranslation()),

                    new Translation2d(MAX_CLEAR_DETECTION_DIST.in(Meters), 0)//other edge of fov
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 2)))
                            .plus(pose.get().getTranslation()),

                    //inner rad
                    new Translation2d(MIN_CLEAR_DETECTION_DIST.in(Meters), 0)//other edge of fov
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 2)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 2.5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 3)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 3.5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 4)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().plus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 6)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation())
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 6)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 4)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 3.5)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 3)))
                            .plus(pose.get().getTranslation()),
                    new Translation2d(MIN_CLEAR_DETECTION_DIST.in(Meters), 0)
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 2.5)))
                            .plus(pose.get().getTranslation()),

                    new Translation2d(MIN_CLEAR_DETECTION_DIST.in(Meters), 0)//one edge of fov
                            .rotateBy(pose.get().getRotation().minus(new Rotation2d(CAMERA_CLEAR_HFOV.in(Radians) / 2)))
                            .plus(pose.get().getTranslation()),
            });
        }

        fuelMap.forEach((coord, set) -> {
            if(set == null || set.isEmpty()){
                return;
            }
            for(int i=0; i< set.size(); i++){
                cellsToRender.add(new Translation3d(coord.x*GRID_SIZE.in(Meters), coord.y*GRID_SIZE.in(Meters), 0.5*i));
            }
        });

        Logger.recordOutput("Subsystems/Fuel vision/cell density map", cellsToRender.toArray(new Translation3d[0]));
        cellsToRender.clear();
    }

    private void deduplicateFuelMap(){
        //for each cell
        for(int i=0; i<fuelMap.size(); i++){
            fuelData[] data = (fuelData[]) fuelMap.values().toArray(new Set[0])[i].toArray(new fuelData[0]);
            //for each fuel in the cell
            for(int j=0; j<data.length; j++){
                fuelData fuel = data[j];
                gridCoord ourCoord = getGridCoord(fuel.pos());
                //for all nearby cells, including this one
                for(int dx=-1; dx<=1; dx++){
                    for(int dy=-1; dy<=1; dy++){
                        Set<fuelData> gridData = fuelMap.get(new gridCoord(ourCoord.x+dx, ourCoord.y+dy));
                        if(gridData == null || gridData.isEmpty()){
                            continue;//cell does not have data or does not exist
                        }
                        //remove if overlapping
                        gridData.removeIf(otherFuel -> fuel.pos.getDistance(otherFuel.pos) < FUEL_OVERLAP_THRESH.in(Meters) && otherFuel != fuel);
                    }
                }
            }
        }
    }

    private void clearFuelInFov(Pose2d pos) {
        fuelMap.forEach((c, set) -> set.removeIf(oldTrack -> isInCameraFOV(oldTrack.pos(), pos) && Logger.getTimestamp()-oldTrack.timestamp > FUEL_POV_CLEAR_GRACE_TIME.in(Microseconds)));
    }

    private void clearFuelInRobot(Pose2d pos) {

        gridCoord ourCoord = getGridCoord(pos.getTranslation());
        //for all nearby cells, including this one
        for(int dx=-1; dx<=1; dx++){
            for(int dy=-1; dy<=1; dy++){
                Set<fuelData> gridData = fuelMap.get(new gridCoord(ourCoord.x+dx, ourCoord.y+dy));
                if(gridData == null || gridData.isEmpty()){
                    continue;//cell does not have data or does not exist
                }
                //remove if overlapping
                gridData.removeIf(otherFuel -> {
                    Translation2d fuelPos = otherFuel.pos().minus(pos.getTranslation()).rotateBy(pos.getRotation());

                    return fuelPos.getX() < ROBOT_WIDTH.in(Meters)/2.0 &&
                            fuelPos.getX() > -ROBOT_WIDTH.in(Meters)/2.0 &&
                            fuelPos.getY() < ROBOT_WIDTH.in(Meters)/2.0 &&
                            fuelPos.getY() > -ROBOT_WIDTH.in(Meters)/2.0;
                });
            }
        }
    }

    /**
     * removes Object o such that {@code Objects.equals(o,fuel)} from the calculated gridCoord from {@code fuel.pos()}
     * @param fuel fuelData object to remove
     * @return true if successful, false otherwise
     */
    private boolean removeFromMap(fuelData fuel){
        gridCoord coord = getGridCoord(fuel.pos());
        if(fuelMap.containsKey(coord)){
            return fuelMap.get(coord).remove(fuel);
        }
        return false;
    }

    private boolean isInCameraFOV(Translation2d fieldPos, Pose2d robotPose) {
        Translation2d cameraOffset = CAMERA_CONFIG.robotToCam.getTranslation().toTranslation2d().rotateBy(robotPose.getRotation());
        Translation2d cameraPos = robotPose.getTranslation().plus(cameraOffset);
        Rotation2d cameraYaw = robotPose.getRotation().plus(new Rotation2d(CAMERA_CONFIG.robotToCam.getRotation().getZ()));

        double distance = cameraPos.getDistance(fieldPos);
        if (distance > MAX_CLEAR_DETECTION_DIST.in(Meters) || distance < MIN_CLEAR_DETECTION_DIST.in(Meters)) {
            return false;
        }

        double angleError = Math.abs(MathUtil.angleModulus(
                ExtraMath.getAngleToPos(fieldPos, cameraPos).in(Radians) - cameraYaw.getRadians()));
        return angleError <= CAMERA_CLEAR_HFOV.in(Radians) / 2.0;
    }

    private void updateFuelPermanence() {
        long now = Logger.getTimestamp();
        fuelMap.forEach((coord, set) -> {
            set.removeIf(fuel -> Microseconds.of(now-fuel.timestamp).gte(FUEL_PERSISTANCE_TIME));
        });
    }

    private void addToMap(fuelData data){
        Set<fuelData> mapCell = fuelMap.computeIfAbsent(getGridCoord(data.pos()), k -> new HashSet<>());
        mapCell.add(data);
    }

    private gridCoord getGridCoord(Translation2d fieldCoord){
        int coordx = (int) (Math.floor(fieldCoord.getX()/GRID_SIZE.in(Meters)));
        int coordy = (int) (Math.floor(fieldCoord.getY()/GRID_SIZE.in(Meters)));
        return new gridCoord(coordx, coordy);
    }

    private Translation2d toFieldRelative(Translation2d pos){
        Translation2d cameraOffset = CAMERA_CONFIG.robotToCam.getTranslation().toTranslation2d().rotateBy(pose.get().getRotation());

        return pos
                .rotateBy(new Rotation2d(pose.get().getRotation().getRadians()))
                .plus(new Translation2d(pose.get().getTranslation().getX(), pose.get().getTranslation().getY()))
                .plus(cameraOffset);
    }
    private Translation2d toFieldRelative(Translation2d pos, Pose2d drivePos){
        Translation2d cameraOffset = CAMERA_CONFIG.robotToCam.getTranslation().toTranslation2d().rotateBy(drivePos.getRotation());

        return pos
                .rotateBy(new Rotation2d(drivePos.getRotation().getRadians()))
                .plus(new Translation2d(drivePos.getTranslation().getX(), drivePos.getTranslation().getY()))
                .plus(cameraOffset);
    }

    public ArrayList<Translation2d> getNewFuel(){
        return newFuel.stream().map(fuelData::pos).collect(Collectors.toCollection(ArrayList::new));
    }

    public ArrayList<Translation2d> getTrackedFuel(){
        return fuelMap.values().stream().flatMap(Collection::stream).map(fuelData::pos).collect(Collectors.toCollection(ArrayList::new));
    }

    public Map<gridCoord, Set<fuelData>> getFuelMap(){
        return fuelMap;
    }

    public record gridCoord(int x, int y){}

    public record fuelData(Translation2d pos, long timestamp){}
}
