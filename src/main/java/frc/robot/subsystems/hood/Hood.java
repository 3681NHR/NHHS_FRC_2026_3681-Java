package frc.robot.subsystems.hood;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.HomeCommand;
import frc.utils.ExtraMath;

import static frc.robot.constants.HoodConstants.*;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
    
    HoodIO io;
    HoodIOInputsAutoLogged in = new HoodIOInputsAutoLogged();

    private final Alert notHomed = new Alert("Hood not homed", Alert.AlertType.kError);

    boolean homing = false;

    private Angle goal;

    public Hood(HoodIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(in);
        Logger.processInputs("IO/Hood", in);
        notHomed.set(!in.homed);

        Logger.recordOutput("Subsystems/Hood/state", getCurrentCommand() == null ? "none" : getCurrentCommand().getName());
        
    }

    /**
     * position control, soft limits apply, and setpoint is clamped
     * @param pos target angle
     * @return Command that drives to pos
     */
    public Command positionControl(Supplier<Angle> pos){
        return Commands.run(() -> {
            this.goal = ExtraMath.clamp(pos.get(), HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
        }).withName("position control");
    }

    public Command instantPositionControl (Supplier<Angle> pos){
        return Commands.run(() -> {
            goal = ExtraMath.clamp(pos.get(), HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
            io.setGoal(goal);
        }, this).withName("position control (instant)");
    }

    /**
     * set openloop vout, soft limits will still apply if homed
     * @param vout - voltage to apply
     * @return Command that runs at voltage
     */
    public Command voltageControl(Supplier<Voltage> vout){
        return Commands.run(() -> {
            io.setVout(vout.get());
            this.goal = null;
        }, this).withName("voltage control");
    }

    /**
     * reset angle to min value and set homed to true
     * @return Command that sets homed to true and resets position to min
     */
    public Command forceHome(){
        return new InstantCommand(() -> {
            io.setHomed(true);
            io.setPos(HOOD_MIN_ANGLE);
        }, this).ignoringDisable(true).withName("force home");
    }

    /**
     * uses voltage commands to auto home
     * @return Command that applies a small voltage until the hood stops,
     */
    public Command home(){

        Command c = new InstantCommand(() -> {
            io.setHomed(false);
            homing = true;
        }).andThen(new HomeCommand(
            HOOD_HOME_VOLTAGE, 
            HOOD_HOME_STOP_TIME, 
            () -> HOOD_HOME_STOP_THRESH.gte(in.velocity),
            v -> io.setVout(v),
            () -> {
                io.setPos(HOOD_MIN_ANGLE);
                io.setHomed(true);
                io.setGoal(HOOD_MIN_ANGLE);
                io.reset();
            })).andThen(new InstantCommand(() -> {
                homing = false;
            }));
        c.addRequirements(this);
        c.setName("auto home");
        return c;
    }

    public Command go(){
        Command c = Commands.run(() -> {
            if (goal != null) {
                io.setGoal(goal);
            }
        });
        c.addRequirements(this);
        c.setName("move to goal");
        return c;
    }
    
    public Angle getAngle(){
        return in.angle;
    }

    public Angle getSetpoint(){
        return in.goal;
    }

    @AutoLogOutput(key="Subsystems/Hood/ready")
    public boolean isReady(){
        return (this.goal != null || in.openloop) && in.homed;
    }

    public boolean isHomed(){
        return in.homed;
    }

    public boolean isHoming(){
        return homing;
    }

    public Command retract(){
        return instantPositionControl(() -> HOOD_MIN_ANGLE);
    }
}
