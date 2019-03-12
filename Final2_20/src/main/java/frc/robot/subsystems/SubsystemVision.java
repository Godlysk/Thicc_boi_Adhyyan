/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.*;

import frc.robot.RobotSettings;
import frc.robot.Utils;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * sexy code
 */

public class SubsystemVision extends Subsystem {
  
  @Override
  public void initDefaultCommand() {
    
  }

//global variables
//------------------
  public double tape1, tape2, targX, distance, servoAng, angle_to_target;
  boolean tape1_is_visible, tape2_is_visible;
  long lastSeenTime;

  public Servo camServo;
//------------------



//constructor
//------------------
  public SubsystemVision(){
    tape1 = RobotSettings.center;
    tape2 = tape1;
    targX = tape2;

    tape1_is_visible = false;
    tape2_is_visible = false;
    lastSeenTime = System.currentTimeMillis();

    servoAng = 90;
    camServo = new Servo(RobotMap.front_cam_servo_port);

    angle_to_target = 0;
  }
//------------------



//get target
//------------------
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
  }
//------------------



//trackServo
//------------------
  public void trackServo(){
    getTarget();
    double error = RobotSettings.center - targX;
    if(tape1_is_visible || tape2_is_visible){            
        servoAng += Math.abs(error)>2?error*0.03:0;
        camServo.setAngle(servoAng);
    }
    else{
        camServo.setAngle(90);
    } 
  }
//------------------



//get angle to target
//------------------
  public void get_angle_to_target() {
    trackServo();
    angle_to_target = (camServo.getAngle()-90)*-1 + (targX-RobotSettings.center)*0.3152;
  }
//------------------



//get snapping angle
//------------------
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
//------------------



//get distance between tapes
//------------------
  public double getDistanceBetweenTapes()
  {
    return (tape1_is_visible && tape2_is_visible)?Math.abs(tape1 - tape2):5;
  }
//------------------



//old get AngleToFollow 12 March
//------------------
  // public double getAngleToFollow(boolean isSlanted){
  //   double snappedAngle = getAngleSnapping(isSlanted);

  //   get_angle_to_target();

  //   double angleToFollow, overshootFactor;

  //   double steerTillThreshold = 30;

  //   if(getDistanceBetweenTapes()>steerTillThreshold){
  //     overshootFactor = 1;
  //     RobotSettings.moveToAngFinal = RobotSettings.moveToAngConstants[1];
  //   }else{
  //     overshootFactor = 4;
  //     RobotSettings.moveToAngFinal = RobotSettings.moveToAngConstants[0];
  //   }

  

  //   if(tape1_is_visible || tape2_is_visible){
  //     angleToFollow = Utils.normaliseHeading(snappedAngle + (((Utils.getCleanedHeading() - snappedAngle) + angle_to_target)*overshootFactor));
  //   }else{
  //     angleToFollow = Utils.getCleanedHeading();
  //   }


  //   double temp = Utils.normaliseHeading(angleToFollow - snappedAngle);
    
  //   temp = Math.signum(temp)*Math.min(Math.abs(temp), 50);

  //   angleToFollow = + temp + snappedAngle ;

  //   return angleToFollow;
  // }
//------------------

//new function
//-------------------
public double getAngleToFollow(boolean isSlanted){
  
    Robot.angleOffset = getAngleSnapping(isSlanted);



    get_angle_to_target();

    double angleToFollow, overshootFactor;

    double steerTillThreshold = 25;

    if(getDistanceBetweenTapes()>steerTillThreshold){
        overshootFactor = 1;
        RobotSettings.moveToAngFinal = RobotSettings.moveToAngConstants[1];
    }else{
        overshootFactor = 4;
        RobotSettings.moveToAngFinal = RobotSettings.moveToAngConstants[0];
    }

    if(tape1_is_visible || tape2_is_visible){
        angleToFollow = Utils.normaliseHeading((Utils.getCleanedHeading() + angle_to_target)*overshootFactor);
    }else{
        angleToFollow = Utils.getCleanedHeading();
    }

    angleToFollow = Math.abs(angleToFollow)>80?Math.signum(angleToFollow)*80:angleToFollow;


    return angleToFollow;
}
//---------------------------



// time out function
//------------------
  public boolean isTimedOut(){
    return (System.currentTimeMillis()-lastSeenTime)>RobotSettings.last_seen_time_out;
  }
//------------------
  




}















// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.command.Subsystem;
// import edu.wpi.first.wpilibj.smartdashboard.*;
// import edu.wpi.first.wpilibj.Servo;
// import frc.robot.RobotSettings;
// import frc.robot.*;

// public class SubsystemVision extends Subsystem{
//     public Servo camServo;
//     public double servoAng;


//     public double angle_to_target; //Angle to target is the angle from the robot to the target(ball or tape)
//     public double targX;

//     double tape1 = RobotSettings.center;
//     double tape2 = RobotSettings.center;
//     public boolean tape1_is_visible = true;
//     public boolean tape2_is_visible = true;

//     public SubsystemVision(){
//         camServo = new Servo(RobotMap.front_cam_servo_port);
//         servoAng = 90;//camServo.getAngle();
//     }

//     public void initDefaultCommand() {
        
//     }

//     public long lastSeenTime = 0; //stores the time when both the tapes were shown

//     public void getTarget(){
//         double [] defaultArray = {-1, -1};

//         double temp1 = SmartDashboard.getNumberArray("tape1", defaultArray)[0];
//         if(temp1 !=-1){
//             tape1 = temp1;
//             tape1_is_visible = true;
//         }else {
//             tape1_is_visible = false;
//         }

//         double temp2 = SmartDashboard.getNumberArray("tape2", defaultArray)[0];
//         if(temp2!=-1){
//             tape2 = temp2;
//             tape1_is_visible = true;
//         }else{
//             tape2_is_visible = false;
//         }

//         if(tape1_is_visible || tape2_is_visible){
//             lastSeenTime = System.currentTimeMillis();
//         }

//         targX = (tape1 + tape2)/2; // average value
//         SmartDashboard.putNumber("diff in tapes", Math.abs(tape1 - tape2));
//         System.out.println(targX);
//     }


//     public void trackServo(){
//         getTarget();
//         double error = RobotSettings.center - targX;
//         if(tape1_is_visible || tape2_is_visible){            
//             servoAng += Math.abs(error)>2?error*0.02:0;
//             camServo.setAngle(servoAng);
//         }else{
//             camServo.setAngle(90);
//         }
//     }



//     public void get_angle_to_target() {
//         trackServo();
//         angle_to_target = (camServo.getAngle()-90)*-1 + (targX-RobotSettings.center)*0.3152;
//     }


//     //computes the angle at which the robot should go in order to become parallel to the target. 
//     //Before running the command, make sure to press button five to reset the angle to 0 when the 
//     //robot is facing the goal. Returns a value between -80 and 80
    
//     public double getAngleSnapping(boolean isSlanted){
//         double[] snapAngle = {0, 90, 180, 270};
//         if(isSlanted) snapAngle = new double[]{30, 150, 210, 330};
        
//         double curHeading = Utils.getCleanedHeading();
//         double angle = snapAngle[0], error = Math.abs(Utils.normaliseHeading(curHeading - snapAngle[0]));
//         for(int i = 1; i < snapAngle.length; i++){
//             if(Math.abs(Utils.normaliseHeading(curHeading - snapAngle[i])) < error){
//                 error = Math.abs(Utils.normaliseHeading(curHeading - snapAngle[i]));
//                 angle = snapAngle[i];
//             } 
//         }
//         SmartDashboard.putNumber("SnapAngle", angle);
//         return angle;
//     }

//     public double distanceTape(){
//         if(tape1_is_visible && tape2_is_visible){
//             return Math.abs(tape1 - tape2);
//         }
//         return -1;
//     }

//     public double getTargFollowAng(boolean isSlanted){   
//         get_angle_to_target();
        
//         double sensetivity = 3;//3; //if the angle relative to the target are too drastic, decrease this. should be greater than 1 though. 
//         double snappedAngle = getAngleSnapping(isSlanted);
        

//         double angleOffset = Utils.normaliseHeading(-snappedAngle+Utils.getCleanedHeading() + angle_to_target);
//         SmartDashboard.putNumber("angle off", angleOffset);
//         double temp = sensetivity*(angleOffset); //main equation. basically takes the absolute heading from the robot to the target and then overshoots in order to become parallel ot the target. 
//         //limit the angle at which the which the robot should move. 
//         if (temp>=80){
//             temp =  80;
//         }else if(temp<=-80){
//             temp =  -80;
//         }

//         if(tape1_is_visible || tape2_is_visible){
//             return Utils.normaliseHeading(temp+snappedAngle);
//             //return Utils.normaliseHeading(temp);
//         }else{
//             System.out.println("Not Visible");
//             return Utils.getCleanedHeading();
//         }
//     }
//     //if the robot is not resposive to the change in angle, go to tankdrive and go to moveToAng function and increase kp


//     public boolean isTimedOut(){
//         return System.currentTimeMillis()-lastSeenTime>RobotSettings.last_seen_time_out;
//     }

// }
