package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.RobotSettings;
import frc.robot.*;

public class SubsystemVision extends Subsystem{
    public static Servo camServo;
    public double servoAng;


    public double angle_to_target; //Angle to target is the angle from the robot to the target(ball or tape)
    public double targX;

    double tape1 = RobotSettings.center;
    double tape2 = RobotSettings.center;
    boolean tape1_is_visible = true;
    boolean tape2_is_visible = true;

    public SubsystemVision(){
        camServo = new Servo(RobotMap.front_cam_servo_port);
        servoAng = camServo.getAngle();
    }

    public void initDefaultCommand() {
    }

    
    public long lastSeenTime = 0; //stores the time when both the tapes were shown

    public void getTarget(){
        double [] defaultArray = {-1, -1};

        double temp1 = SmartDashboard.getNumberArray("tape1", defaultArray)[0];
        if(temp1 !=-1){
            tape1 = temp1;
        }else {
            tape1_is_visible = false;
        }

        double temp2 = SmartDashboard.getNumberArray("tape2", defaultArray)[0];
        if(temp2!=-1){
            tape2 = temp2;
        }else{
            tape2_is_visible = false;
        }

        if(tape1_is_visible || tape2_is_visible){
            lastSeenTime = System.currentTimeMillis();
        }

        targX = (tape1 + tape2)/2; // average value
    }


    public void trackServo(){
        getTarget();
        double error = RobotSettings.center - targX;
        if(tape1_is_visible || tape2_is_visible){            
            servoAng += Math.abs(error)>3?error*0.03:0;
        }
        camServo.setAngle(servoAng);
    }



    public void get_angle_to_target() {
        trackServo();
        angle_to_target = (camServo.getAngle()-90)*-1 + (targX-RobotSettings.center)*0.3152;
    }


    //computes the angle at which the robot should go in order to become parallel to the target. 
    //Before running the command, make sure to press button five to reset the angle to 0 when the 
    //robot is facing the goal. Returns a value between -80 and 80
    
    public double getTargFollowAng(){   
        get_angle_to_target();
        double sensetivity = 2.5; //if the angle relative to the target are too drastic, decrease this. should be greater than 1 though. 
        double temp = sensetivity*(Utils.getCleanedHeading() + angle_to_target); //main equation. basically takes the absolute heading from the robot to the target and then overshoots in order to become parallel ot the target. 
        //limit the angle at which the which the robot should move. 
        if (temp>=80){
            temp =  80;
        }else if(temp<=-80){
            temp =  -80;
        }

        return temp;
    }
    //if the robot is not resposive to the change in angle, go to tankdrive and go to moveToAng function and increase kp


    public boolean isTimedOut(){
        return System.currentTimeMillis()-lastSeenTime>RobotSettings.last_seen_time_out;
    }

}
