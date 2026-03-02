package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import frc.utils.ExtraMath;

public class launchLUT {
    public static final double[][] LUTGround = {
    //  dist, hood, speed, time
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0}
    };
    public static final double[][] LUTHub = {
    //  dist, hood, speed, time
        {Units.feetToMeters(3.5), Units.degreesToRadians(20), 3107.0, 0.0},
        {Units.feetToMeters(4), Units.degreesToRadians(20), 2928.0, 0.0},
        {Units.feetToMeters(6), Units.degreesToRadians(20), 3178.0, 0.0},
        {Units.feetToMeters(7.5), Units.degreesToRadians(20), 3321.0, 0.0},
        {Units.feetToMeters(8.5), Units.degreesToRadians(30), 3190.0, 0.0},
        {Units.feetToMeters(10), Units.degreesToRadians(30), 3250.0, 0.0},
        {Units.feetToMeters(11.5), Units.degreesToRadians(30), 3500.0, 0.0},
        {Units.feetToMeters(13), Units.degreesToRadians(30), 3750.0, 0.0},
        {Units.feetToMeters(15), Units.degreesToRadians(35), 3857.0, 0.0},
        {Units.feetToMeters(18), Units.degreesToRadians(45), 3857.0, 0.0},
    };

    /**
     * get data from LUT at given distance val
     * @param dist - value to lookup
     * @param lerp - weather to lerp between closest values or return nearest entry(will always return above dist)
     * @param LUT - 2d array to use, needs to be n by 4 and sorted by distance
     * @return - array with data, [hood angle, launcher speed, TOF]
     * <p> edge cases:
     * <p> - when dist is greater than the farthest entry, farthest entry will be returned. 
     *          if lerp is true, returned value will be extrapolated from highest two entries
     * <p> - when dist is closer than minimum entry, the minumum entry will be returned
     */
    public static double[] get(double dist, boolean lerp, double[][] LUT){
        double[] out = new double[3];

        int i=0;
        while(LUT[i][0]<dist){
            i++;
            if(i>=LUT.length){
                if(!lerp){
                    return new double[]{     
                        LUT[LUT.length-1][1],
                        LUT[LUT.length-1][2],
                        LUT[LUT.length-1][3]
                    };
                } else {
                    return new double[]{    //extrapolate based on last two values
                        ExtraMath.lerp(LUT[LUT.length-2][1], LUT[LUT.length-1][1], (dist-LUT[LUT.length-2][0])/(LUT[LUT.length-1][0]-LUT[LUT.length-2][0])),
                        ExtraMath.lerp(LUT[LUT.length-2][2], LUT[LUT.length-1][2], (dist-LUT[LUT.length-2][0])/(LUT[LUT.length-1][0]-LUT[LUT.length-2][0])),
                        ExtraMath.lerp(LUT[LUT.length-2][3], LUT[LUT.length-1][3], (dist-LUT[LUT.length-2][0])/(LUT[LUT.length-1][0]-LUT[LUT.length-2][0]))
                    };
                }
            }
        }
        if(lerp && i>0){
            out[0] = ExtraMath.lerp(LUT[i-1][1], LUT[i][1], (dist-LUT[i-1][0])/(LUT[i][0]-LUT[i-1][0]));
            out[1] = ExtraMath.lerp(LUT[i-1][2], LUT[i][2], (dist-LUT[i-1][0])/(LUT[i][0]-LUT[i-1][0]));
            out[2] = ExtraMath.lerp(LUT[i-1][3], LUT[i][3], (dist-LUT[i-1][0])/(LUT[i][0]-LUT[i-1][0]));
        } else {
            out[0] = LUT[i][1];
            out[1] = LUT[i][2];
            out[2] = LUT[i][3];
        }
        return out;
    }
}
