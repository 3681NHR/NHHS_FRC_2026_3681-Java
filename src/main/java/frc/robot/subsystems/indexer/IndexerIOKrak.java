package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.utils.motorWrappers.SparkMax;
import frc.utils.motorWrappers.TalonFX;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.IndexerConstants.*;
import static frc.utils.SparkUtil.tryUntilOk;

public class IndexerIOKrak implements IndexerIO {

    private final TalonFX motor = new TalonFX(INDEXER_MOTOR_ID);
    private final StatusSignal<AngularVelocity> speed = motor.getVelocity();
    private final StatusSignal<Voltage>         voltageOut = motor.getMotorVoltage();
    private final StatusSignal<Current>         currentOut = motor.getSupplyCurrent();
    private final StatusSignal<Temperature>     temp = motor.getDeviceTemp();
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final Alert motorDisconnectAlert = new Alert("Indexer motor disconnected!", AlertType.kError);

    public IndexerIOKrak() {

        SparkMaxConfig kickerConfig = new SparkMaxConfig();
        kickerConfig.inverted(INDEXER_MOTOR_INVERT).idleMode(IdleMode.kBrake).voltageCompensation(12)
                .smartCurrentLimit((int) INDEXER_MAX_CURRENT.in(Amps));
        kickerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        kickerConfig.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR).velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

        tryUntilOk(motor, 5, () -> motor.configure(kickerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    }

    @Override
    public void updateInputs(IndexerIOInputs input) {

        input.speed = speed.getValue();

        input.motorCurrentOut = currentOut.getValue();
        input.motorVoltageOut = voltageOut.getValue();
        input.motorTemp = temp.getValue();

        input.motorConnected = motor.isConnected();

        motorDisconnectAlert.set(!input.motorConnected);
    }

    @Override
    public void setVout(Voltage vout) {
        motor.setControl(voltageControl.withOutput(vout));
    }
}
