package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;



public class Utils {
    public static AHRS navx = new AHRS(SPI.Port.kMXP);
    //public static AnalogInput ultra = new AnalogInput(0);

    
    public static double normaliseHeading(double angle){
        //parameter: any angle
        //return: returns an angle between -180 and 180
        while(angle>180){
            angle -= 360;
        }
        while(angle < -180){
            angle += 360;
        }

        return angle;
    }



    public static double getCleanedHeading()
    {
        double temp = navx.getFusedHeading() - Robot.angleOffset;
        return normaliseHeading(temp);
    }

    //returns a value which is in the range of lower and upper after taking abs
    public static double inAbsRange(double value, double lower, double upper){
        double sign = Math.signum(value);
        value = Math.abs(value);
        lower = Math.abs(lower);
        upper = Math.abs(upper);
        if(value < lower) value = lower;
        if(value > upper) value = upper;
        return value*sign;
      }


}