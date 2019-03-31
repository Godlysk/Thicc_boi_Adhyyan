/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Arm.ClawArm.*;
import frc.robot.Arm.LiftArm.*;
import frc.robot.Arm.RotateArm.*;
import frc.robot.Chassis.*;
import frc.robot.Vision.*;

import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
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

//declaring all subsystems
//---------------------------
  public static SubsystemTankDrive tankDriveSubsystem;
  public static SubsystemVision utilvisionSubsystem;
  public static SubsystemVision visionSubsystem;
  public static SubsystemArmLift armLiftSubsystem;
  public static SubsystemRotateArm rotateArmSubsystem;
  public static SubsystemSolenoidArm solenoidArmSubsystem;
  public static SubsystemArmWheels armShooterWheels;

  public static OI oi;
  public static Joy1 joystick1;
  public static Joy2 joystick2;

  public static NetworkTable table;

  public static Compressor compressor;

  public static PowerDistributionPanel pdp;
  public static UsbCamera usb;
//---------------------------

//roboInit
//---------------------------
  @Override
  public void robotInit() {
    visionSubsystem = new SubsystemVision();
    tankDriveSubsystem = new SubsystemTankDrive();
    armLiftSubsystem = new SubsystemArmLift();
    rotateArmSubsystem = new SubsystemRotateArm();
    solenoidArmSubsystem = new SubsystemSolenoidArm();
    armShooterWheels = new SubsystemArmWheels();
    pdp = new PowerDistributionPanel();
    oi = new OI();
    joystick1 = new Joy1();
    joystick2 = new Joy2();
    usb = CameraServer.getInstance().startAutomaticCapture(); 
    angleOffset = 0;
  }
//---------------------------

  public static double angleOffset;
  
  @Override
  public void robotPeriodic() {
  
    //Exposure Mode Switching 
    //-----------------------
    if(oi.joy2.getRawButton(joystick2.exposureButton)){
      SmartDashboard.putNumber("ExpAuto", 0.00);
    }else{
      SmartDashboard.putNumber("ExpAuto", 1.00);
    }
    //-----------------------


    SmartDashboard.putBoolean("Tapes Visible?", visionSubsystem.bothTapesVis());

    SmartDashboard.putData(visionSubsystem);
    SmartDashboard.putData(rotateArmSubsystem);
    //SmartDashboard.putData(compressor);
    SmartDashboard.putNumber("Arm Rotate Angle", rotateArmSubsystem.getAngle());
    SmartDashboard.putBoolean("Arm Navx connected", rotateArmSubsystem.armNavx.isConnected());
    SmartDashboard.putBoolean("Chassis Navx connected", Utils.navx.isConnected());
    SmartDashboard.putBoolean("Arm Open", solenoidArmSubsystem.ds.get() == Value.kForward);
    SmartDashboard.putNumber("fusedHeading", Utils.getCleanedHeading());

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
    tankDriveSubsystem.steer_corr = 0;
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
    tankDriveSubsystem.steer_corr = 0;
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
