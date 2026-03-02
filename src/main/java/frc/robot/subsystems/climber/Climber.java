package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;

public class Climber extends SubsystemBase {
    
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged in = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(in);
        Logger.processInputs("IO/Climber", in);
        Logger.recordOutput("Subsystems/Climber/state", (getCurrentCommand() == null ? "none" : getCurrentCommand().getName()));
    }
    /**
     * extends the climber to 1.0 (placeholder value, will be tuned later)
     */
    public Command extend() {
        // FIXME: placeholder value
        return runOnce(() -> io.setSetpoint(ClimbConstants.EXTEND_POSITION));
    }
    /**
     * retracts the climber to 0
     */
    public Command retract() {
        return runOnce(() -> io.setSetpoint(0.0));
    }
    /**
     * set encoder position to 0 (for button)
     */
    public Command zeroEncoder() {
        return runOnce(() -> io.zeroEncoder());
    }
}
