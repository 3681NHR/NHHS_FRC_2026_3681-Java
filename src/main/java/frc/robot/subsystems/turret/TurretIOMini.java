package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.TurretConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkLowLevel;
import frc.utils.motorWrappers.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import frc.utils.controlWrappers.ProfiledPID;
import frc.utils.controlWrappers.SimpleFF;

public class TurretIOMini implements TurretIO {

    private SparkMax motor = new SparkMax(TURRET_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private CANcoder encoder = new CANcoder(TURRET_ENCODER_1_ID);
    
    private Angle goal = Radians.of(0.0);
    private boolean openLoop = false;
    private Voltage Vout = Volts.of(0.0);
    private Angle angle = Radians.of(0.0);

    private final LinearSystem<N2, N1, N2> model = LinearSystemId.identifyPositionSystem(TURRET_ID_GAINS.kV, TURRET_ID_GAINS.kA);
    private final KalmanFilter<N2, N1, N2> filter = new KalmanFilter<N2, N1, N2>(Nat.N2(), Nat.N2(), model, VecBuilder.fill(1, 1), VecBuilder.fill(0.001, 0.001), 0.02);

    private ProfiledPID pid = new ProfiledPID(TURRET_PID_GAINS);
    private SimpleFF ff = new SimpleFF(TURRET_FF_GAINS);

    private Alert disconenct = new Alert("turret absolute encoder is disconnected", AlertType.kWarning);
    private Alert motorError = new Alert("", AlertType.kError);

    public TurretIOMini(){
        TURRET_PID_GAINS.withCallback(() -> {
            pid.setGains(TURRET_PID_GAINS);
        });
        TURRET_FF_GAINS.withCallback(() -> {
            ff.setKs(TURRET_FF_GAINS.kS);
            ff.setKv(TURRET_FF_GAINS.kV);
            ff.setKa(TURRET_FF_GAINS.kA);
        });
    }

    @Override
    public void updateInputs(TurretIOInputs input){
        disconenct.set(!encoder.isConnected());
        motorError.set(motor.getLastError() != REVLibError.kOk);
        motorError.setText("turret motor error: " + motor.getLastError().toString());

        filter.predict(VecBuilder.fill(Vout.in(Volts) - Math.min(TURRET_ID_GAINS.kS, Math.abs(Vout.in(Volts)))*Math.signum(getSpeed().magnitude())), Constants.EVENT_LOOP_TIME);
        filter.correct(VecBuilder.fill(Vout.in(Volts) - Math.min(TURRET_ID_GAINS.kS, Math.abs(Vout.in(Volts)))*Math.signum(getSpeed().magnitude())), VecBuilder.fill(getAngle().in(Radians), getSpeed().in(RadiansPerSecond)));
        angle = getAngle();

        if(!openLoop){
            Vout = Volts.of(pid.calculate(getAngle().in(Radians), goal.in(Radians)));
            Vout = Vout.plus(Volts.of(ff.calculate(pid.getSetpoint().velocity)));
        }
        motor.setVoltage(Vout);
        
        if(!DriverStation.isEnabled()){
            motor.stopMotor();
        }
        
        input.angle = getAngle();
        input.speed = getSpeed();

        input.motorVoltageOut = Vout;

        input.goal = goal;
        input.setpointPos = Radians.of(pid.getSetpoint().position);
        input.setpointVel = RadiansPerSecond.of(pid.getSetpoint().velocity);
        input.atSetpoint = MathUtil.isNear(goal.in(Radians), angle.in(Radians), TURRET_SETPOINT_TOLERANCE.in(Radians));
    
        input.openLoop = openLoop;
    }

    @Override
    public void setGoal(Angle goal){
        this.openLoop = false;
        this.goal = goal;
    }
    @Override
    public void setVout(Voltage vout){
        this.openLoop = true;
        Vout = vout;
    }
    
    private Angle getAngle(){
            // return Radians.of(filter.getXhat().get(0,0));
            return encoder.getPosition().getValue();
    }
    private AngularVelocity getSpeed(){
            return encoder.getVelocity().getValue();
        
    }
}
