package frc.robot;

public class RobotSettings
{
    public static double[] ysens = {0.65, 0.65};
    public static double[] zsens = {1.5, 0.8};

    public static double acceleration_max = 0.5;

    public static double moveToAngConstants[] = {0.003, 0.004};
    public static double moveToAngFinal;


    //deadbands
    public static double zthresh = 0.08;

    public static double zthresh_steer = 0.05;

    //pixel center of the camera
    public static double center = 80;


    //Autonomous Related settings

    //timeout limit to end the autonomous command
    public static long last_seen_time_out = 500;

    //speed at which servo tracks the tagret with the camera
    public static double servo_track_sens = 0.03;
    
    //Joystick sensetivity at which the robot will approach the hatch dock.
    public static double autonomousDockSens = 0.3;


    public static double USDigitalConstant = 225;



    public static double tapeUpper[]  = { 0,0,0};
}


