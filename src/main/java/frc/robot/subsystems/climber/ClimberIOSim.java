package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ClimbConstants.CLIMB_PID_GAINS;
import static frc.robot.constants.ClimbConstants.FF;
import static frc.robot.constants.ClimbConstants.CLIMB_ID_GAINS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.constants.Constants;
import frc.utils.controlWrappers.ElevatorFF;
import frc.utils.controlWrappers.ProfiledPID;

public class ClimberIOSim implements ClimberIO {

    private final LinearSystem<N2, N1, N2> model = LinearSystemId.identifyPositionSystem(CLIMB_ID_GAINS.kV,
            CLIMB_ID_GAINS.kA);
    private final LinearSystemSim<N2, N1, N2> sim = new LinearSystemSim<N2, N1, N2>(model, 0.01, 0.1);
    private final KalmanFilter<N2, N1, N2> filter = new KalmanFilter<N2, N1, N2>(Nat.N2(), Nat.N2(), model,
            VecBuilder.fill(0.2, 1.0), VecBuilder.fill(1.3, 0.7), Constants.EVENT_LOOP_TIME);

    private final ProfiledPID pid = new ProfiledPID(CLIMB_PID_GAINS);
    private final ElevatorFF ff = new ElevatorFF(FF);

    private boolean openLoop = false;
    private double goalMeters = 0.0;
    private double appliedVolts = 0.0;
    private double position = 0.0;

    @Override
    public void updateInputs(ClimberIOInputs input) {
        sim.update(Constants.EVENT_LOOP_TIME);

        double voltageInput = appliedVolts
                - Math.min(CLIMB_ID_GAINS.kS, Math.abs(appliedVolts)) * Math.signum(sim.getOutput().get(1, 0));

        filter.predict(VecBuilder.fill(voltageInput), Constants.EVENT_LOOP_TIME);
        filter.correct(VecBuilder.fill(voltageInput), sim.getOutput());
        position = filter.getXhat().get(0, 0);

        if (!openLoop) {
            appliedVolts = pid.calculate(position, goalMeters) + ff.calculate(pid.getSetpoint().velocity);
        }

        appliedVolts = MathUtil.clamp(appliedVolts, -RobotController.getBatteryVoltage(),
                RobotController.getBatteryVoltage());

        if (DriverStation.isEnabled()) {
            sim.setInput(appliedVolts
                    - Math.min(CLIMB_ID_GAINS.kS, Math.abs(appliedVolts)) * Math.signum(sim.getOutput().get(1, 0)));
        } else {
            sim.setInput(-Math.min(CLIMB_ID_GAINS.kS, Math.abs(appliedVolts)) * Math.signum(sim.getOutput().get(1, 0)));
        }

        input.motorVoltageOut = Volts.of(appliedVolts);
        input.motorTemp = Celsius.of(25.0);
        input.encoderPosition = Meters.of(position);
        input.encoderVelocity = MetersPerSecond.of(filter.getXhat(1));
        input.climbVelocitySetpoint = MetersPerSecond.of(pid.getSetpoint().velocity);
        input.climbPositionSetpoint = Meters.of(pid.getSetpoint().position);
        input.connected = true;
        input.goal = Meters.of(goalMeters);
        input.atSetpoint = pid.atSetpoint();
        input.openLoop = openLoop;
    }

    /** sets the goal position for the climber. */
    @Override
    public void setSetpoint(double position) {
        openLoop = false;
        goalMeters = position;
    }

    @Override
    public void zeroEncoder() {
        sim.setState(VecBuilder.fill(0.0, filter.getXhat(1)));
        filter.setXhat(VecBuilder.fill(0.0, filter.getXhat(1)));
        goalMeters = 0.0;
        pid.reset(0.0);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        openLoop = true;
        appliedVolts = voltage.in(Volts);
    }
}
