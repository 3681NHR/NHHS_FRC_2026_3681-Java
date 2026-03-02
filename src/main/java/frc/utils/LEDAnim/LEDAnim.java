package frc.utils.LEDAnim;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.Constants;
import frc.utils.ExtraMath;

public class LEDAnim {
    public Color[] leds;
    public Color[] initLeds;

    private double scroll = 0;
    private double scrollShift;

    public LEDAnim(int length){
        leds = new Color[length];
        for(int i=0; i<length; i++){
            leds[i] = Color.kBlack;
        }
    }
    public LEDAnim(int length, Color c){
        leds = new Color[length];
        for(int i=0; i<length; i++){
            leds[i] = c;
        }
    }
    public LEDAnim(Color[] c){
        leds = c;
        initLeds = c.clone();
    }

    public Color[] getLEDs(){
        return leds;
    }
    public void update(){
        if(scroll != 0){
            scrollShift += scroll * Constants.EVENT_LOOP_TIME;
            leds = offset(new LEDAnim(initLeds), scrollShift, true).getLEDs();
        }
    }

    /**
     * scroll at reletive speed
     * @param speed leds per second
     */
    public LEDAnim scroll(double speed){
        scroll = speed;
        return this;
    }
    /**
     * multiplies rgb values of base and mask
     * @param base
     * @param mask
     */
    static LEDAnim mask(LEDAnim base, LEDAnim mask){
        Color[] result = new Color[base.leds.length];
        for(int i=0; i<base.leds.length; i++){
            double r = (base.leds[i].red * mask.leds[i].red);
            double g = (base.leds[i].green * mask.leds[i].green);
            double b = (base.leds[i].blue * mask.leds[i].blue);
            result[i] = new Color(r, g, b);
        }

        return new LEDAnim(result);
    }

    public static LEDAnim offset(LEDAnim in, double offset, boolean wrap){
        Color[] result = new Color[in.leds.length];
        for(int i=0; i<in.leds.length; i++){
            if(wrap){
                double a = (i + offset) % in.leds.length;
                
                result[i] = ExtraMath.colLerp(in.leds[(int)a], in.leds[(int)(a+1)%in.leds.length], a - (int)a);
            } else {
                if(offset < 0 || offset >= in.leds.length){
                    result[i] = Color.kBlack;
                } else {
                    result[i] = ExtraMath.colLerp(in.leds[(int)(i - offset)], in.leds[(int)(i - offset + 1) % in.leds.length], (i - offset) - (int)(i - offset));
                }
            }
        }
        return new LEDAnim(result);
    }
}
