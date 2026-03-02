package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.DriveConstants.*;
import static frc.utils.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.utils.motorWrappers.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.DriveConstants.module;
import frc.utils.SparkOdometryThread;
import frc.utils.controlWrappers.ProfiledPID;
import frc.utils.controlWrappers.SimpleFF;

import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max
 * turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOSpark implements ModuleIO {

    // Hardware objects
    private final SparkBase driveSpark;
    private final SparkBase turnSpark;
    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController driveController;
    private final ProfiledPID turnPID = new ProfiledPID(module.TURN_PID);
    private final SimpleFF turnFF = new SimpleFF(module.TURN_FF);
    private final SimpleFF driveFF = new SimpleFF(module.DRIVE_FF);

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    private Angle turnGoal = Radians.of(0.0);
    private AngularVelocity driveGoal = RadiansPerSecond.of(0.0);

    private boolean driveClosedLoop = true;
    private boolean turnClosedLoop = true;

    private Angle drivePositionRad = Radians.of(0.0);
    private Angle turnPositionRad = Radians.of(0.0);

    private AngularVelocity driveVelocityRadPerSecond = RadiansPerSecond.of(0.0);
    private AngularVelocity turnVelocity = RadiansPerSecond.of(0.0);

	public ModuleIOSpark(int IO) {
        driveSpark = new SparkMax(
                switch (IO) {
                    case 0 -> module.FL_DRIVE_ID;
                    case 1 -> module.FR_DRIVE_ID;
                    case 2 -> module.BL_DRIVE_ID;
                    case 3 -> module.BR_DRIVE_ID;
                    default -> 0;
                },
                MotorType.kBrushless);
        turnSpark = new SparkMax(
                switch (IO) {
                    case 0 -> module.FL_TURN_ID;
                    case 1 -> module.FR_TURN_ID;
                    case 2 -> module.BL_TURN_ID;
                    case 3 -> module.BR_TURN_ID;
                    default -> 0;
                },
                MotorType.kBrushless);
        driveEncoder = driveSpark.getEncoder();
        turnEncoder = turnSpark.getAbsoluteEncoder();
        driveController = driveSpark.getClosedLoopController();

        // Configure drive motor
        var driveConfig = new SparkMaxConfig();
        driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit((int)module.DRIVE_MAX_CURRENT.in(Amps))
                .voltageCompensation(12.0)
                .inverted(module.DRIVE_INVERT);
        driveConfig.encoder
                .positionConversionFactor(module.DRIVE_ENCODER_POS_FACTOR)
                .velocityConversionFactor(module.DRIVE_ENCODER_VEL_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(4);
                
        driveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                        module.DRIVE_PID.kP,
                        module.DRIVE_PID.kI,
                        module.DRIVE_PID.kD);
        driveConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int)(1000 / ODOMETRY_FREQ.in(Hertz)))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                driveSpark,
                5,
                () -> driveSpark.configure(
                        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

        // Configure turn motor
        var turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(module.TURN_INVERT)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit((int)module.TURN_CURRENT_LIM.in(Amps))
                .voltageCompensation(12.0);
        turnConfig.absoluteEncoder
                .inverted(module.TURN_ENCODER_INVERT)
                .positionConversionFactor(module.TURN_ENCODER_POS_FACTOR)
                .velocityConversionFactor(module.TURN_ENCODER_VEL_FACTOR)
                .averageDepth(8);
        turnConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        turnConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQ.in(Hertz)))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                turnSpark,
                5,
                () -> turnSpark.configure(
                        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        turnPID.enableContinuousInput(module.TURN_MIN_POS.in(Radians), module.TURN_MAX_POS.in(Radians));
        // Create odometry queues
        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        drivePositionRad = Radians.of(driveEncoder.getPosition());

        driveVelocityRadPerSecond = RadiansPerSecond.of(driveEncoder.getVelocity());
        turnVelocity = RadiansPerSecond.of(turnEncoder.getVelocity());

        inputs.turnTemp = Celsius.of(turnSpark.getMotorTemperature());
        inputs.driveTemp = Celsius.of(driveSpark.getMotorTemperature());

        // Update drive inputs
        sparkStickyFault = false;
        ifOk(driveSpark, () -> drivePositionRad.in(Radians), (value) -> inputs.drivePosition = Radians.of(value));
        ifOk(driveSpark, () -> driveVelocityRadPerSecond.in(RadiansPerSecond), (value) -> inputs.driveVelocity = RadiansPerSecond.of(value));
        ifOk(
                driveSpark,
                new DoubleSupplier[] { driveSpark::getAppliedOutput, driveSpark::getBusVoltage },
                (values) -> inputs.driveAppliedVolts = Volts.of(values[0] * values[1]));
        ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrent = Amps.of(value));
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);
        inputs.driveGoal = driveGoal;
        inputs.driveSetpoint = driveGoal;

        // Update turn inputs
        sparkStickyFault = false;
        ifOk(
                turnSpark,
                () -> turnPositionRad.in(Radians),
                (value) -> inputs.turnPosition = Radians.of(value));
        ifOk(turnSpark, () -> turnVelocity.in(RadiansPerSecond), (value) -> inputs.turnVelocity = RadiansPerSecond.of(value));
        ifOk(
                turnSpark,
                new DoubleSupplier[] { turnSpark::getAppliedOutput, turnSpark::getBusVoltage },
                (values) -> inputs.turnAppliedVolts = Volts.of(values[0] * values[1]));
        ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrent = Amps.of(value));
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);
        inputs.turnGoal = turnGoal;
        inputs.turnSetpoint = Radians.of(turnPID.getSetpoint().position);

        // Update odometry inputs
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble(e -> e).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble(e -> e).toArray();
        inputs.odometryTurnPositionsRad = turnPositionQueue.stream().mapToDouble(e -> e).toArray();
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();

        if (driveClosedLoop) {
            // drivesetpoint.position is actually velocity
            double ffVolts = driveFF.calculate(driveGoal.in(RadiansPerSecond));
            driveController.setSetpoint(
                    driveGoal.in(RadiansPerSecond) + getDriveOffsetVelocity(),
                    ControlType.kVelocity,
                    ClosedLoopSlot.kSlot0,
                    ffVolts,
                    ArbFFUnits.kVoltage);
        }
        if (turnClosedLoop) {
            double ffVolts = turnFF.calculate(turnPID.getSetpoint().velocity);
            turnPID.setGoal(turnGoal.in(Radians));
            turnSpark.setVoltage(ffVolts + turnPID.calculate(turnEncoder.getPosition()));
        }

    }
    public double getDriveOffsetVelocity() {
        return turnEncoder.getVelocity() * module.DRIVE_OFFSET_VEL_FACTOR;
    }

    @Override
    public void setDriveOpenLoop(Voltage output) {
        driveSpark.setVoltage(output);
        driveClosedLoop = false;
    }

    @Override
    public void setTurnOpenLoop(Voltage output) {
        turnSpark.setVoltage(output);
        turnClosedLoop = false;
    }

    @Override
    public void setDriveVelocity(AngularVelocity velocity) {
        driveGoal = velocity;
        driveClosedLoop = true;
    }

    @Override
    public void setTurnPosition(Angle rotation) {
        turnGoal = Radians.of(MathUtil.inputModulus(rotation.in(Radians), module.TURN_MIN_POS.in(Radians), module.TURN_MAX_POS.in(Radians)));
        turnClosedLoop = true;
    }
}