package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.utils.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.IndexerConstants;
import frc.utils.controlWrappers.SimpleFF;

public class IndexerIOReal implements IndexerIO {
    private final SparkBase indexerSpark;
    private final RelativeEncoder indexerEncoder;

    private final SparkClosedLoopController indexerController;
    private final SimpleFF indexerFF = new SimpleFF(IndexerConstants.INDEXER_FF_GAINS);
    
    AngularVelocity goal = RPM.of(0);
    Voltage vout = Volts.of(0.0);
    
    Alert overheatAlert = new Alert("", AlertType.kError);
    
    private boolean openLoop = false;

    public IndexerIOReal() {
        indexerSpark = new SparkMax(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
        indexerEncoder = indexerSpark.getEncoder();
        indexerController = indexerSpark.getClosedLoopController();

        SparkMaxConfig indexerConfig = new SparkMaxConfig();
        indexerConfig.inverted(IndexerConstants.INVERT).idleMode(IdleMode.kCoast).voltageCompensation(12).smartCurrentLimit((int) IndexerConstants.CURRENT_LIMIT.in(Amps));
        indexerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(IndexerConstants.INDEXER_PID_GAINS.kP, IndexerConstants.INDEXER_PID_GAINS.kI, IndexerConstants.INDEXER_PID_GAINS.kD);
        tryUntilOk(indexerSpark, 5, () -> indexerSpark.configure(indexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(IndexerIOInputs input) {
        input.speed = RPM.of(indexerEncoder.getVelocity());

        input.motorCurrentOut = Amps.of(indexerSpark.getOutputCurrent());
        input.motorVoltageOut = vout;
        input.motorTemp = Celsius.of(indexerSpark.getMotorTemperature());

        input.goal = goal;
        input.openLoop = openLoop;

        if (DriverStation.isEnabled()) {
            if (!openLoop) {
                double ffVoltage = indexerFF.calculate(input.goal.in(RadiansPerSecond));
                indexerController.setSetpoint(goal.in(RadiansPerSecond), ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVoltage, ArbFFUnits.kVoltage);
            } else {
                indexerController.setSetpoint(vout.in(Volts), ControlType.kVoltage);
            }
        } else {
            indexerSpark.stopMotor();
        }

        if (input.motorTemp.gt(IndexerConstants.MAX_TEMP)) {
            overheatAlert.setText("Indexer motor overheat: " + input.motorTemp.in(Celsius) + " *C !");
            overheatAlert.set(true);
        } else {
            overheatAlert.set(false);
        }
    }

    @Override
    public void setVoltage(Voltage vout) {
        this.openLoop = true;
        this.vout = vout;
    }

    @Override
    public void setGoal(AngularVelocity goal) {
        this.openLoop = false;
        this.goal = goal;
    }

    @Override
    public void setOpenLoop(boolean openLoop) {
        this.openLoop = openLoop;
    }
}
