package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Ultility class that keeps track of who can score when and for how long (dependent on who won auto)
 */
public class ShiftTracker{
    private static MatchPhase phase = MatchPhase.UNKNOWN;
    private static final Timer matchTimer = new Timer();

    private static Alliance ourAlliance = null;
    private static Alliance allianceThatWonAuto = null;

    /* This constant stands for the amount of time that you can shoot before it's legal
     * And still have the fuel count in the hub
    */
    private static final double SCORING_DELAY_SECONDS = 0.5;

    //Just in case we start teleop without auto and we need to correctly offset the time
    private static double timeOffset = 0;

    public static void start() {
        reset();

        if (DriverStation.isAutonomous())
            phase = MatchPhase.AUTO;
        else {
            phase = MatchPhase.TRANSITION_SHIFT;

            double autoDuration = (MatchPhase.AUTO.getEndTime() - MatchPhase.AUTO.getStartTime());
            double autoToTeleopDuration = (MatchPhase.AUTO_TELE_TRANSITION.getEndTime() - MatchPhase.AUTO_TELE_TRANSITION.getStartTime());
            timeOffset = autoDuration + autoToTeleopDuration;
        }

        matchTimer.start();
    }

    public static void reset() {
        matchTimer.reset();
        matchTimer.stop();

        timeOffset = 0;
    }

    public static MatchPhase getCurrentMatchPhase() {
        return phase;
    }

    /**
     * Called periodically to ensure we accurately track which shift we are in
     */
    public static void update() {
        if (allianceThatWonAuto == null) {
            String speculatedAutoWinner = DriverStation.getGameSpecificMessage();
            if (!speculatedAutoWinner.isEmpty())
                allianceThatWonAuto = (speculatedAutoWinner.equals("R")) ? Alliance.Red : Alliance.Blue;
        }

        if (ourAlliance == null)
            ourAlliance = DriverStation.getAlliance().orElse(null);

        if (isRunning()) {
            if (phase != MatchPhase.UNKNOWN && phase != null) {
                if (getTime() > phase.getEndTime())
                    phase = phase.getNext();
            } else {
                /* Phase becomes null when we are in auto or teleop DS modes and have exceeded the standard
                * match duration. We stop tracking shifts and any calls to canScore() will return true for testing purposes
                */
                reset();
                phase = MatchPhase.UNKNOWN;
            }

            if(RobotBase.isSimulation()){
                //activate both hubs in sim when they should be
                ((Arena2026Rebuilt) Arena2026Rebuilt.getInstance()).setShouldRunClock(!(
                        phase == MatchPhase.UNKNOWN
                        || phase == MatchPhase.AUTO
                        || phase == MatchPhase.AUTO_TELE_TRANSITION
                        || phase == MatchPhase.TRANSITION_SHIFT
                        || phase == MatchPhase.ENDGAME));
            }
        }

        Logger.recordOutput("shift/who won auto", allianceThatWonAuto);
        Logger.recordOutput("shift/we won auto", weWonAuto());
        Logger.recordOutput("shift/can score", canScore());
        Logger.recordOutput("shift/time left in shift", getTimeLeftInShift());
        Logger.recordOutput("shift/current phase", getCurrentMatchPhase());
    }

    /**
     * @return Whether or not we can score into our alliance's hub
     */
    public static boolean canScore() {
        if (!matchTimer.isRunning())
            return true;
        else if (ourAlliance == null)
            return false;

        switch (phase) {
            case AUTO, ENDGAME, UNKNOWN, AUTO_TELE_TRANSITION -> { return true;}

            case TRANSITION_SHIFT -> {
                if (weWonAuto()) return getTimeLeftInShift() >= SCORING_DELAY_SECONDS;
                else return true;
            }

            case SHIFT1, SHIFT3 -> {
                if (weWonAuto()) return getTimeLeftInShift() <= SCORING_DELAY_SECONDS;
                else return getTimeLeftInShift() >= SCORING_DELAY_SECONDS;
            }

            case SHIFT2 -> {
                if (weWonAuto()) return getTimeLeftInShift() >= SCORING_DELAY_SECONDS;
                else return getTimeLeftInShift() <= SCORING_DELAY_SECONDS;
            }

            case SHIFT4 -> {
                if(!weWonAuto()) return getTimeLeftInShift() <= SCORING_DELAY_SECONDS;
                else return true;
            }

            default -> { return false; }
        }
    }

    public static boolean isRunning() {
        return matchTimer.isRunning();
    }

    private static boolean weWonAuto() {
        return ourAlliance == allianceThatWonAuto;
    }

    public static double getTimeLeftInShift() {
        return Math.max(0, phase.getEndTime() - getTime());
    }

    private static double getTime() {
        return matchTimer.get() + timeOffset;
    }
}