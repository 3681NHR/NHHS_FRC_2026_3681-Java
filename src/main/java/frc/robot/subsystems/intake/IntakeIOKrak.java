package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.utils.controlWrappers.ArmFF;
import frc.utils.controlWrappers.ProfiledPID;
import frc.utils.motorWrappers.SparkMax;
import frc.utils.motorWrappers.TalonFX;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.IntakeConstants.*;

public class IntakeIOKrak implements IntakeIO {

    //  Roller
    private final TalonFX rollerMotor = new TalonFX(INTAKE_ROLLER_MOTOR_ID);
    private final StatusSignal<Voltage> rollerMotorVoltage = rollerMotor.getMotorVoltage();
    private final StatusSignal<Temperature> rollerMotorTemp = rollerMotor.getDeviceTemp();
    private final StatusSignal<Current> rollerMotorCurrent = rollerMotor.getSupplyCurrent();
    private final StatusSignal<AngularVelocity> rollerMotorSpeed = rollerMotor.getVelocity();

    private final VoltageOut rollerMotorVoltageControl = new VoltageOut(0);
    private final Alert rollerMotorDisconnect = new Alert("Intake roller Spark disconnected!", AlertType.kError);

    //  Pivot
    private final SparkMax pivotMotor = new SparkMax(INTAKE_PIVOT_MOTOR_ID, MotorType.kBrushless);
    private final CANcoder pivotEncoder = new CANcoder(INTAKE_PIVOT_ENCODER_ID);

    private final ProfiledPID pivotPID = new ProfiledPID(INTAKE_PIVOT_PID_GAINS);
    private ArmFF pivotFF = new ArmFF(INTAKE_PIVOT_FF_GAINS);

    private final Alert pivotMotorDisconnect = new Alert("Intake pivot Spark disconnected!", AlertType.kError);
    private final Alert pivotEncoderDisconnect = new Alert("Intake pivot encoder disconnected!", AlertType.kError);

    private boolean pivotOpenLoop = false;
    private Angle pivotGoal = INTAKE_STOWED_ANGLE;

    public IntakeIOKrak() {
        // Live-tuning callbacks
        INTAKE_PIVOT_PID_GAINS.withCallback(() -> pivotPID.setGains(INTAKE_PIVOT_PID_GAINS));
        INTAKE_PIVOT_FF_GAINS.withCallback(() -> {
            pivotFF.setKs(INTAKE_PIVOT_FF_GAINS.kS);
            pivotFF.setKg(INTAKE_PIVOT_FF_GAINS.kG);
            pivotFF.setKv(INTAKE_PIVOT_FF_GAINS.kV);
            pivotFF.setKa(INTAKE_PIVOT_FF_GAINS.kA);
        });

        pivotPID.setTolerance(INTAKE_PIVOT_TOLERANCE.in(Units.Radians));

        // Roller motor config
//        SparkMaxConfig rollerCfg = new SparkMaxConfig();
//        rollerCfg.idleMode(IdleMode.kBrake)
//                 .inverted(INTAKE_ROLLER_INVERTED)
//                 .smartCurrentLimit((int)INTAKE_ROLLER_CURRENT_LIM.in(Amps));
        TalonFXConfiguration rollerCfg = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(INTAKE_PIVOT_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(INTAKE_ROLLER_CURRENT_LIM));

//        rollerMotor.configure(rollerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Pivot motor config
        SparkMaxConfig pivotCfg = new SparkMaxConfig();
        pivotCfg.idleMode(IdleMode.kBrake)
                .inverted(INTAKE_PIVOT_INVERTED)
                .smartCurrentLimit((int)INTAKE_PIVOT_CURRENT_LIM.in(Amps));
        pivotMotor.configure(pivotCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
    }

    @Override
    public void updateInputs(IntakeIOInputs input) {
        //  Roller closed-loop
        input.rollerVelocity = rollerMotorSpeed.getValue();

        input.rollerVoltageOut = rollerMotorVoltage.getValue();
        input.rollerCurrentOut = rollerMotorCurrent.getValue();
        input.rollerTemp = rollerMotorTemp.getValue();
        
        input.rollerConnected = rollerMotor.isConnected();

        //  Pivot closed-loop 
        if (!pivotOpenLoop) {
            double pid = pivotPID.calculate(pivotEncoder.getAbsolutePosition().getValue().in(Units.Radians), pivotGoal.in(Units.Radians));
            double ff = pivotFF.calculate(pivotEncoder.getAbsolutePosition().getValue().in(Radians), pivotPID.getSetpoint().velocity);
            pivotMotor.setVoltage(Units.Volts.of(pid + ff));
        }
        input.pivotAngle = pivotEncoder.getAbsolutePosition().getValue();
        input.pivotVelocity = pivotEncoder.getVelocity().getValue();

        input.pivotGoal = pivotGoal;
        input.pivotSetpointPos = Units.Radians.of(pivotPID.getSetpoint().position);
        input.pivotSetpointVel = Units.RadiansPerSecond.of(pivotPID.getSetpoint().velocity);
        input.pivotAtSetpoint = pivotPID.atSetpoint();

        input.pivotVoltageOut = Units.Volts.of(pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage());
        input.pivotCurrentOut = Units.Amps.of(pivotMotor.getOutputCurrent());
        input.pivotTemp = Units.Celsius.of(pivotMotor.getMotorTemperature());
        
        input.pivotMotorConnected = pivotMotor.getLastError() != REVLibError.kCANDisconnected;
        input.pivotEncoderConnected = pivotEncoder.isConnected();
        input.pivotOpenLoop = pivotOpenLoop;
        pivotMotorDisconnect.set(!input.pivotMotorConnected);
        pivotEncoderDisconnect.set(!input.pivotEncoderConnected);

    }

    @Override
    public void setRollerVoltage(Voltage voltage) {
        rollerMotor.setControl(rollerMotorVoltageControl.withOutput(voltage));
    }

    @Override
    public void setPivotGoal(Angle goal) {
        pivotOpenLoop = false;
        pivotGoal = goal;
    }

    @Override
    public void setPivotVoltage(Voltage voltage) {
        pivotOpenLoop = true;
        pivotMotor.setVoltage(voltage);
    }

}
