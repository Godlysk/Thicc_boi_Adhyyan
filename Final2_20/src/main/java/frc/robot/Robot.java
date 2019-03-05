/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Arm.ClawArm.*;
import frc.robot.Arm.LiftArm.*;
import frc.robot.Arm.RotateArm.*;
import frc.robot.Chassis.*;
import frc.robot.RobotLifter.SubsystemRobotLifterForward;
import frc.robot.RobotLifter.SubsystemRobotLifterUp;
import frc.robot.subsystems.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.PowerDistributionPanel;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

//instantiating all subsystems
//---------------------------
  public static SubsystemTankDrive tankDriveSubsystem;
  public static OI oi;
  public static SubsystemVision utilvisionSubsystem;
  public static SubsystemVision visionSubsystem;
  public static SubsystemArmLift armLiftSubsystem;
  public static SubsystemRotateArm rotateArmSubsystem;
  public static SubsystemSolenoidArm solenoidArmSubsystem;
  public static SubsystemRobotLifterUp lifterUpSubsystem;
  public static SubsystemRobotLifterForward lifterForwardSubsystem;
  public static SubsystemArmWheels armShooterWheels;
  public static Joy1 joystick1;
  public static Joy2 joystick2;



  //public static NetworkTable table;


  public static PowerDistributionPanel pdp = new PowerDistributionPanel();
//---------------------------


  @Override
  public void robotInit() {
    visionSubsystem = new SubsystemVision();
    tankDriveSubsystem = new SubsystemTankDrive();
    armLiftSubsystem = new SubsystemArmLift();
    rotateArmSubsystem = new SubsystemRotateArm();
    solenoidArmSubsystem = new SubsystemSolenoidArm();
    lifterUpSubsystem = new SubsystemRobotLifterUp();
    armShooterWheels = new SubsystemArmWheels();
    //lifterForwardSubsystem = new SubsystemRobotLifterForward(); 
    oi = new OI();
    joystick1 = new Joy1();
    joystick2 = new Joy2();
    //table = NetworkTable.getTable("SmartDashboard");
  }

  boolean preExpButton = false;
  boolean ExposureSetting = true;
  
  @Override
  public void robotPeriodic() {
    
//Exposure Mode Switching 
    boolean tempButton = Robot.oi.joy1.getRawButton(Robot.joystick1.expButton);

    if(tempButton && !preExpButton)
    {
      ExposureSetting = !ExposureSetting;
    }
    preExpButton = tempButton;
//-------------------------

//Drive sensetivity mode switching
  if(oi.joy1.getRawButton(2)){
    RobotSettings.ysens = 0.3;
    RobotSettings.zsens = 0.2;
  } else{
    RobotSettings.ysens = 0.6;
    RobotSettings.zsens = 2;
  }
//-------------------------





  }


//disabled period
//-----------------------
  @Override
  public void disabledInit() {
    
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }
//-----------------------

  



//autonomous period
//-----------------------
  @Override
  public void autonomousInit() {
  }
  
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }
//-----------------------



  //teleop phase
//-------------------------
  @Override
  public void teleopInit() {

  }
  


  @Override
  public void teleopPeriodic() {
    
    Scheduler.getInstance().run();
  }

//-------------------------


//test periodic
//-----------------------

  @Override
  public void testPeriodic() {
  }

//-----------------------
}
