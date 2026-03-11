package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.utils.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.IndexerConstants;

public class IndexerIOReal implements IndexerIO {
    private final SparkBase indexerSpark;
    private final RelativeEncoder indexerEncoder;

    private final SparkClosedLoopController indexerController;
    
    Voltage vout = Volts.of(0.0);
    
    public IndexerIOReal() {
        indexerSpark = new SparkMax(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
        indexerEncoder = indexerSpark.getEncoder();
        indexerController = indexerSpark.getClosedLoopController();

        SparkMaxConfig indexerConfig = new SparkMaxConfig();
        indexerConfig.inverted(IndexerConstants.INVERT).idleMode(IdleMode.kCoast).voltageCompensation(12).smartCurrentLimit((int) IndexerConstants.CURRENT_LIMIT.in(Amps));
        indexerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        tryUntilOk(indexerSpark, 5, () -> indexerSpark.configure(indexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(IndexerIOInputs input) {
        input.speed = RPM.of(indexerEncoder.getVelocity());

        input.motorCurrentOut = Amps.of(indexerSpark.getOutputCurrent());
        input.motorVoltageOut = vout;
        input.motorTemp = Celsius.of(indexerSpark.getMotorTemperature());

        if (DriverStation.isEnabled()) {
            indexerController.setSetpoint(vout.in(Volts), ControlType.kVoltage);
        } else {
            indexerSpark.stopMotor();
        }
    }

    @Override
    public void setVout(Voltage vout) {
        this.vout = vout;
    }
}
