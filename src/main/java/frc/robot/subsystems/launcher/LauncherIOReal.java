package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.constants.LauncherConstants.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.utils.motorWrappers.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class LauncherIOReal implements LauncherIO {

    TalonFX motor = new TalonFX(LAUNCHER_MOTOR_ID);

    VoltageOut openLoopRequest = new VoltageOut(0);
    VelocityVoltage closedLoopRequest = new VelocityVoltage(0);
    
    boolean openLoop = false;

    Alert overheat = new Alert("", AlertType.kError);//dynamic text, dont set here
    Alert disconnect = new Alert("Launcher motor disconnected!", AlertType.kError);
    
    public LauncherIOReal(){
        motor.getConfigurator()
        .apply(new Slot0Configs()
            .withKP(LAUNCHER_PID_GAINS.kP)
            .withKI(LAUNCHER_PID_GAINS.kI)
            .withKD(LAUNCHER_PID_GAINS.kD)
            .withKS(LAUNCHER_FF_GAINS.kS)
            .withKV(LAUNCHER_FF_GAINS.kV)
            .withKA(LAUNCHER_FF_GAINS.kA)
        );
        motor.getConfigurator().apply(new VoltageConfigs().withPeakReverseVoltage(0));
    }

    @Override
    public void updateInputs(LauncherIOInputs input){

        motor.stopMotor();

        if(motor.getDeviceTemp().getValue().magnitude() > LAUNCHER_MAX_TEMP.magnitude()){
            overheat.set(true);
            overheat.setText("Launcher motor overheat! ("+motor.getDeviceTemp().getValue().in(Celsius)+"C)");
        } else {
            overheat.set(false);
        }
        disconnect.set(motor.isConnected());
        
        input.angle = motor.getPosition().getValue();
        input.speed = motor.getVelocity().getValue();

        input.motorCurrentOut = motor.getStatorCurrent().getValue();
        input.motorTemp = motor.getDeviceTemp().getValue();
        input.motorVoltageOut = motor.getMotorVoltage().getValue();

        input.goal = closedLoopRequest.getVelocityMeasure();
        input.atSetpoint = MathUtil.isNear(closedLoopRequest.getVelocityMeasure().in(RPM), motor.getVelocity().getValue().in(RPM), LAUNCHER_SETPOINT_TOLERANCE.in(RPM));
    
        input.openLoop = openLoop;
    }

    @Override
    public void setGoal(AngularVelocity goal){
        this.openLoop = false;
        motor.setControl(closedLoopRequest.withVelocity(goal));
    }
    @Override
    public void setVout(Voltage vout){
        this.openLoop = true;
        motor.setControl(openLoopRequest.withOutput(vout));
    }
    
}
