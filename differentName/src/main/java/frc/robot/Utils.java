package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Encoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.AnalogInput;



public class Utils {
    public static AHRS navx = new AHRS(SPI.Port.kMXP);
    public static AnalogInput ultra = new AnalogInput(0);
        

    public static double getUltra(){

        return (ultra.getVoltage()/0.0048)*0.5;
    }

    
    public static double getCleanedHeading()
    {
        double temp = navx.getFusedHeading();
        if(temp>180)
        {
        temp= temp-360;
        }
        return temp;
    }
}