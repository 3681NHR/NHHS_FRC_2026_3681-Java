package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LaunchLUT.ShotParams;
import frc.robot.subsystems.swerve.Drive;
import frc.utils.ExtraMath;

public final class SOTMSolver extends SubsystemBase{
    private static SOTMSolver instance;
    private ShotParams params = new ShotParams(Meters.of(0), Radians.of(0), RPM.of(0), Seconds.of(0));
    private Translation2d target = new Translation2d();
    private Translation2d originalTarget = target;
    private Drive drive;
    private Angle turretAngle = Radians.of(0);

    private Translation2d operatorTrim = new Translation2d();

    private SOTMSolver(){
    };

    public synchronized static SOTMSolver getInstance(){
        if(instance == null){
            instance = new SOTMSolver();
        }
        return instance;
    }

    @Override 
    public void periodic(){
        calculate();
    }

    public void setTarget(Translation2d targ){
        this.target = targ.plus(operatorTrim);
        this.originalTarget = targ;
    }

    /**
     * must call whenever the operator trim is updated
    */
    public void setOperatorTrim(Translation2d operatorTrim) {
        this.operatorTrim = operatorTrim;
        this.setTarget(originalTarget);
    }

    public void setDrive(Drive drive){
        this.drive = drive;
    }
    public void calculate(){
        Translation2d curr = drive.getPose().getTranslation();

        double dist = curr.getDistance(target);
        double newDist = 0;
        Translation2d vec = new Translation2d();
        params = LaunchLUT.get(Meters.of(dist), true, LaunchLUT.LUTHub);
        
        for(int i=0; i<5 && Math.abs(dist-newDist) > Units.inchesToMeters(10); i++){
            dist = newDist;
            
            vec = curr.plus(new Translation2d(
                drive.getChassisSpeeds().vxMetersPerSecond * params.time().in(Seconds),
                drive.getChassisSpeeds().vyMetersPerSecond * params.time().in(Seconds)
                ));
            
            turretAngle = ExtraMath.getAngleToPos(target, vec);
            newDist = vec.getDistance(target);
            params = LaunchLUT.get(Meters.of(newDist), true, LaunchLUT.LUTHub);
        }
        
    }
    /**
     * calculate
     * <p><p>
     * requires the following:
     * <p> - slope of time is never 0
     * <p> - time is never 0
     */
    public void calculate2(){
        Translation2d curr = drive.getPose().getTranslation();

        params = LaunchLUT.get(Meters.of(curr.getDistance(target)), true, LaunchLUT.LUTHub);
        if(params.time().in(Seconds) <= 0){
            System.err.println("SOTM calculation canceled(time of flight was 0)");
            return;
        }

        Translation2d vel = new Translation2d(
            drive.getChassisSpeeds().vxMetersPerSecond,
            drive.getChassisSpeeds().vyMetersPerSecond
        );

        Translation2d shotVel = new Translation2d(
            Math.cos(ExtraMath.getAngleToPos(target, curr).in(Radians)),
            Math.sin(ExtraMath.getAngleToPos(target, curr).in(Radians))
        ).times(curr.getDistance(target) / params.time().in(Seconds));
        
        Translation2d targetVel = shotVel.minus(vel);
        if(targetVel.getDistance(new Translation2d()) == 0){
            //point at target as fallback
            turretAngle = ExtraMath.getAngleToPos(target, curr);
        } else {
            turretAngle = Radians.of(targetVel.getAngle().getRadians());
        }

        double targetV = targetVel.getDistance(new Translation2d());
        double currentV = params.dist().in(Meters) / params.time().in(Seconds);
        double dist = params.dist().in(Meters);

        for(int i=0; i<10 && Math.abs(currentV - targetV) > 0.005; i++){
            // d/dx(dist/time)
            // = dist'*time - dist*time' / time^2
            // = -dist*time' / time^2
            // -dist feels wrong
            double dv = (-dist*LaunchLUT.getSlope(Meters.of(dist), LaunchLUT.LUTHub).time().in(Seconds))/Math.pow(params.time().in(Seconds), 2);
            
            //newtons method the goat
            dist -= (currentV-targetV)/dv;

            params = LaunchLUT.get(Meters.of(dist), true, LaunchLUT.LUTHub);
            
            if(params.time().in(Seconds) <= 0){
                System.err.println("SOTM calculation canceled(time of flight was 0)");
                return;
            }
            currentV = dist/params.time().in(Seconds);
        }
    }

    public ShotParams getParams(boolean refresh){
        if(refresh){
            calculate();
        }
        return params;
    }
    public Angle getAngle(boolean refresh){
        if(refresh){
            calculate();
        }
        return turretAngle;
    }

    public Translation2d getTarget(){
        return target;
    }
}
