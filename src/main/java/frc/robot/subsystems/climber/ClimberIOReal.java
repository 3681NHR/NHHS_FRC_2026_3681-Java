package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.utils.motorWrappers.SparkMax;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.ClimbConstants;
import frc.utils.controlWrappers.ProfiledPID;
import frc.utils.controlWrappers.ElevatorFF;

public class ClimberIOReal implements ClimberIO {
    private final SparkMax doom = new SparkMax(ClimbConstants.MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder despair = doom.getEncoder();
    private final ProfiledPID pid = new ProfiledPID(ClimbConstants.CLIMB_PID_GAINS);
    private final ElevatorFF ff = new ElevatorFF(ClimbConstants.FF);
    private boolean openLoop = false;
    private Alert disconnect = new Alert("climber Spark is disconnected, id: " + ClimbConstants.MOTOR_ID, AlertType.kError);
    private double goal = 0.0;
    
    public ClimberIOReal() {
        ClimbConstants.CLIMB_PID_GAINS.withCallback(() -> {
            pid.setGains(ClimbConstants.CLIMB_PID_GAINS);
        });
        ClimbConstants.FF.withCallback(() -> {
            ff.setKs(ClimbConstants.FF.kS);
            ff.setKv(ClimbConstants.FF.kV);
            ff.setKa(ClimbConstants.FF.kA);
            ff.setKg(ClimbConstants.FF.kG);
        });
        

        SparkMaxConfig doomConfig = new SparkMaxConfig();

        // motor configuration
        doomConfig.idleMode(IdleMode.kBrake).inverted(ClimbConstants.INVERTED);
        // relative encoder configuration
        doomConfig.encoder.positionConversionFactor(ClimbConstants.POSITION_CONVERSION_FACTOR).velocityConversionFactor(ClimbConstants.VELOCITY_CONVERSION_FACTOR);

        doom.configure(doomConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    @Override
    public void updateInputs(ClimberIOInputs input){
        if (!openLoop) {
            doom.setVoltage(Units.Volts.of(pid.calculate(despair.getPosition(), goal) + ff.calculate(pid.getSetpoint().velocity)));
        }
        input.motorCurrentOut = Units.Amps.of(doom.getOutputCurrent());
        input.motorTemp = Units.Celsius.of(doom.getMotorTemperature());
        input.encoderPosition = Units.Meters.of(despair.getPosition());
        input.encoderVelocity = Units.MetersPerSecond.of(despair.getVelocity());
        input.goal = Units.Meters.of(goal);
        input.atSetpoint = pid.atSetpoint();
        input.connected = doom.getLastError() != REVLibError.kCANDisconnected;
        input.motorVoltageOut = Units.Volts.of(doom.getAppliedOutput() * doom.getBusVoltage());
        input.openLoop = openLoop;
        input.climbVelocitySetpoint = Units.MetersPerSecond.of(pid.getSetpoint().velocity);
        input.climbPositionSetpoint = Units.Meters.of(pid.getSetpoint().position);
        disconnect.set(!input.connected);
    }

    /** sets the voltage for the climber. */
    @Override
    public void setVoltage(Voltage voltage){
        openLoop = true;
        doom.setVoltage(voltage);
    }
    /** sets the goal position for the climber. */
    @Override
    public void setSetpoint(double position){
        openLoop = false;
        goal = position;
    }
    /**
     * zeros the encoder position. (for button)
     */
    @Override
     public void zeroEncoder() {
        despair.setPosition(0);
    }
}

