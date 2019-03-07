package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.RobotSettings;
import frc.robot.*;

public class SubsystemVision extends Subsystem{
    public Servo camServo;
    public double servoAng;


    public double angle_to_target; //Angle to target is the angle from the robot to the target(ball or tape)
    public double targX;

    double tape1 = RobotSettings.center;
    double tape2 = RobotSettings.center;
    public boolean tape1_is_visible = true;
    public boolean tape2_is_visible = true;

    public SubsystemVision(){
        camServo = new Servo(RobotMap.front_cam_servo_port);
        servoAng = 90;//camServo.getAngle();
    }

    public void initDefaultCommand() {
        
    }

    public long lastSeenTime = 0; //stores the time when both the tapes were shown

    public void getTarget(){
        double [] defaultArray = {-1, -1};

        double temp1 = SmartDashboard.getNumberArray("tape1", defaultArray)[0];
        if(temp1 !=-1){
            tape1 = temp1;
            tape1_is_visible = true;
        }else {
            tape1_is_visible = false;
        }

        double temp2 = SmartDashboard.getNumberArray("tape2", defaultArray)[0];
        if(temp2!=-1){
            tape2 = temp2;
            tape1_is_visible = true;
        }else{
            tape2_is_visible = false;
        }

        if(tape1_is_visible || tape2_is_visible){
            lastSeenTime = System.currentTimeMillis();
        }

        targX = (tape1 + tape2)/2; // average value

        System.out.println(targX);
    }


    public void trackServo(){
        getTarget();
        double error = RobotSettings.center - targX;
        if(tape1_is_visible || tape2_is_visible){            
            servoAng += Math.abs(error)>1?error*0.03:0;
            camServo.setAngle(servoAng);
        }
        else{
            camServo.setAngle(90);
        }
        
    }



    public void get_angle_to_target() {
        trackServo();
        angle_to_target = (camServo.getAngle()-90)*-1 + (targX-RobotSettings.center)*0.3152;
    }


    //computes the angle at which the robot should go in order to become parallel to the target. 
    //Before running the command, make sure to press button five to reset the angle to 0 when the 
    //robot is facing the goal. Returns a value between -80 and 80
    
    public double getAngleSnapping(boolean isSlanted){
        double[] snapAngle = {0, 90, 180, 270};
        if(isSlanted) snapAngle = new double[]{30, 150, 210, 330};
        
        double curHeading = Utils.getCleanedHeading();
        double angle = snapAngle[0], error = Math.abs(Utils.normaliseHeading(curHeading - snapAngle[0]));
        for(int i = 1; i < snapAngle.length; i++){
            if(Math.abs(Utils.normaliseHeading(curHeading - snapAngle[i])) < error){
                error = Math.abs(Utils.normaliseHeading(curHeading - snapAngle[i]));
                angle = snapAngle[i];
            } 
        }
        SmartDashboard.putNumber("SnapAngle", angle);
        return angle;
    }

    public double getTargFollowAng(boolean isSlanted){   
        get_angle_to_target();
        double sensetivity = 3.4; //if the angle relative to the target are too drastic, decrease this. should be greater than 1 though. 
        double snappedAngle = getAngleSnapping(isSlanted);
        double temp = sensetivity*Utils.normaliseHeading(-snappedAngle+Utils.getCleanedHeading() + angle_to_target); //main equation. basically takes the absolute heading from the robot to the target and then overshoots in order to become parallel ot the target. 
        //double temp = sensetivity*(Utils.getCleanedHeading()+ angle_to_target); //main equation. basically takes the absolute heading from the robot to the target and then overshoots in order to become parallel ot the target. 
        //limit the angle at which the which the robot should move. 
        if (temp>=80){
            temp =  80;
        }else if(temp<=-80){
            temp =  -80;
        }

        if(tape1_is_visible || tape2_is_visible){
            return Utils.normaliseHeading(temp+snappedAngle);
        }
        else{
            System.out.println("Not Visible");
            return Utils.getCleanedHeading();
        }
    }
    //if the robot is not resposive to the change in angle, go to tankdrive and go to moveToAng function and increase kp


    public boolean isTimedOut(){
        return System.currentTimeMillis()-lastSeenTime>RobotSettings.last_seen_time_out;
    }

}
