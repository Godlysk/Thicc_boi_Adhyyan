/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  public static int front_cam_servo_port = 0;

  public static int FR_port = 3;
  public static int FL_port = 2;
  public static int BR_port = 4;
  public static int BL_port = 1;


  public static int liftMotor_port = 11;
  public static int rotateArm_port = 10;

  //subsystemRobotLifterup
  public static int robotLifterUpMotor1 = 6;
  public static int robotLifterUpMotor2 = 7;

  //subsystemRobotLifterForward
  // public static int robotLifterForwardMotor = 5;

  //shooter motors
  public static int shooterMotorLeft = 9;
  public static int shooterMotorRight = 8;
  
  //PCM Ports
  public static int pistonPortFirst = 6;
  public static int pistonPortSecond = 4;

  //Encoder Ports
  public static int chassisEncoder_r = 0;
  public static int chassisEncoder_l = 2;
  public static int elevatorEncoder_l = 6;
  public static int elevatorEncoder_r = 10;
  public static int rotateArmEncoder = 8;
  public static int liftArmEncoder = 4;


}