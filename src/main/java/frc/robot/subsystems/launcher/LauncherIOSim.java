package frc.robot.subsystems.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.constants.Constants;
import frc.utils.controlWrappers.PID;
import frc.utils.controlWrappers.SimpleFF;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.LauncherConstants.*;

public class LauncherIOSim implements LauncherIO {

    AngularVelocity goal = RPM.of(0.0);
    Voltage vout = Volts.of(0.0);
    AngularVelocity speed = RPM.of(0.0);
    boolean openLoop = false;

    private PID pid = new PID(LAUNCHER_PID_GAINS);
    private SimpleFF ff = new SimpleFF(LAUNCHER_FF_GAINS);
    
    private final LinearSystem<N2, N1, N2> model = LinearSystemId.identifyPositionSystem(LAUNCHER_ID_GAINS.kV, LAUNCHER_ID_GAINS.kA);
    private final LinearSystemSim<N2, N1, N2> sim = new LinearSystemSim<N2, N1, N2>(model, 0.01, 0.1);
    @Override
    public void updateInputs(LauncherIOInputs input){
        sim.update(Constants.EVENT_LOOP_TIME);
        filter.predict(VecBuilder.fill(vout.in(Volts) - Math.min(LAUNCHER_ID_GAINS.kS, Math.abs(vout.in(Volts)))*Math.signum(sim.getOutput().get(1,0))), Constants.EVENT_LOOP_TIME);
        filter.correct(VecBuilder.fill(vout.in(Volts) - Math.min(LAUNCHER_ID_GAINS.kS, Math.abs(vout.in(Volts)))*Math.signum(sim.getOutput().get(1,0))), sim.getOutput());
        speed = RadiansPerSecond.of(filter.getXhat().get(1,0));

        if(!openLoop){
            vout = Volts.of(pid.calculate(speed.in(RPM), goal.in(RPM)));
            vout = vout.plus(Volts.of(ff.calculate(goal.in(RPM))));
            //set min out to 0v
            vout = Volts.of(Math.max(0, vout.in(Volts)));
        }
        if(DriverStation.isEnabled()){
            sim.setInput(vout.in(Volts) - Math.min(LAUNCHER_ID_GAINS.kS, Math.abs(vout.in(Volts)))*Math.signum(sim.getOutput().get(1,0)));
        } else {
            sim.setInput(0);
        }
        
        input.angle = Radians.of(sim.getOutput().get(0,0));
        input.speed = speed;

        input.motorVoltageOut = vout;

        input.goal = goal;
        input.atSetpoint = MathUtil.isNear(goal.in(RPM), speed.in(RPM), LAUNCHER_SETPOINT_TOLERANCE.in(RPM));
    
        input.openLoop = openLoop;
    }

    @Override
    public void setGoal(AngularVelocity goal){
        this.openLoop = false;
        this.goal = goal;
    }
    @Override
    public void setVout(Voltage vout){
        this.openLoop = true;
        this.vout = vout;
        this.goal = RPM.of(0);
    }

}