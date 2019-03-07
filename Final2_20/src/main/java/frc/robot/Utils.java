package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.AnalogInput;



public class Utils {
    public static AHRS navx = new AHRS(SPI.Port.kMXP);
    public static AnalogInput ultra = new AnalogInput(0);
        

    public static double getUltra(){

        return (ultra.getVoltage()/0.0048)*0.5;
    }

    
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
        double temp = navx.getFusedHeading();
        return normaliseHeading(temp);
        
    }
}