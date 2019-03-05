/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Chassis;

import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils;
import frc.robot.RobotSettings;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.*;
/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class SubsystemTankDrive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  public Encoder enc_r = new Encoder(RobotMap.chassisEncoder_r, (RobotMap.chassisEncoder_r +1),true, Encoder.EncodingType.k4X);
  public Encoder enc_l = new Encoder(RobotMap.chassisEncoder_l, (RobotMap.chassisEncoder_l+1), false, Encoder.EncodingType.k4X);

  public WPI_TalonSRX FR = new WPI_TalonSRX(RobotMap.FR_port);
  public WPI_TalonSRX BR = new WPI_TalonSRX(RobotMap.BR_port);
  public WPI_TalonSRX FL = new WPI_TalonSRX(RobotMap.FL_port);
  public WPI_TalonSRX BL = new WPI_TalonSRX(RobotMap.BL_port);  

  public SubsystemTankDrive(){
    FR = new WPI_TalonSRX(RobotMap.FR_port);
    BR = new WPI_TalonSRX(RobotMap.BR_port);
    FL = new WPI_TalonSRX(RobotMap.FL_port);
    BL = new WPI_TalonSRX(RobotMap.BL_port);  

    FR.setInverted(true);
    BR.setInverted(true);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CommandSteerDrive());
  }

//SteerDrive
//---------------------------------
  double steer_corr = 0;
  double z_deadband = 0.04;
  public void steerDrive(double yaxis, double zaxis)
  {
    z_deadband = 0.04 + Math.abs(yaxis) * 0.05;

    SmartDashboard.putNumber("yaxis_joy", yaxis);
    SmartDashboard.putNumber("zaxis_joy", zaxis);

    if(Math.abs(zaxis)<0.03){
      zaxis=0;
    }
    double err, leftSpeed, rightSpeed;
    err = Utils.navx.getRate() - mapAngRate(zaxis);
    double p_corr = 0.25*err;
    steer_corr += err*0.02;
    leftSpeed =  yaxis*RobotSettings.ysens + (-1*(steer_corr + p_corr));
    rightSpeed = yaxis*RobotSettings.ysens+ (steer_corr + p_corr);

    drive(leftSpeed, rightSpeed);
  }
//---------------------------------
  




//PIDRetardedDrive
//-------------------------------------
  
  // double d_gain = 0.01;
  // double i_gain = 0.0;
  // double p_gain = 0;//0.0000;
  
  // double s_d_corr, s_i_corr = 0, s_p_corr = 0;
  // double t_corr=0;
  // double pre_temp = 0;
  // double err = 0;
  // public void PIDRetardedDrive(){
  //   double rightSpeed , leftSpeed;
  //   double yaxis = Robot.oi.getY(Robot.oi.joy1);
  //   double zaxis = Robot.oi.getZ(Robot.oi.joy1);

  //   SmartDashboard.putNumber("Rate of Rotation", Utils.navx.getRate());

      
  //   if(Math.abs(yaxis) > Math.abs(zaxis)){
  //     err = enc_l.getRate() - enc_r.getRate();
  //     s_d_corr = Utils.navx.getRate() * d_gain;
  //     s_i_corr += err*i_gain;
  //     s_p_corr = err*p_gain;

  //     rightSpeed = yaxis*RobotSettings.ysens + s_d_corr + s_i_corr + s_p_corr;
  //     leftSpeed = yaxis*RobotSettings.ysens - s_d_corr - s_i_corr - s_p_corr;

  //     SmartDashboard.putNumber("icorrection", s_i_corr);
  //     SmartDashboard.putNumber("dCorrection", s_d_corr);

  //   }else if(Math.abs(yaxis)<Math.abs(zaxis) && Math.abs(zaxis)>RobotSettings.zthresh){
  //     err = Utils.navx.getRate() - mapAngRate(zaxis);
  //     double p_corr = 0.2*err;
  //     t_corr+=err*0.02;

  //     leftSpeed =  -1*(t_corr + p_corr);
  //     rightSpeed = (t_corr + p_corr);
  //   }
  //   else
  //   {
  //     t_corr = 0;
  //     s_d_corr = 0;
  //     rightSpeed = 0;
  //     leftSpeed = 0;
  //   }
  //   drive(leftSpeed, rightSpeed);
  // }
//-------------------------------------

public void PIDRetardedDrive(double yaxis, double zaxis){
    double rightSpeed , leftSpeed;

    SmartDashboard.putNumber("Rate of Rotation", Utils.navx.getRate());

      
    if(Math.abs(yaxis) > Math.abs(zaxis)){
      zaxis = 0;
    }else if(Math.abs(yaxis)<Math.abs(zaxis) && Math.abs(zaxis)>RobotSettings.zthresh){
      yaxis = 0;
    }
    else
    {
      zaxis = 0;
      yaxis = 0;
    }
    z_deadband = 0.04 + Math.abs(yaxis) * 0.05;

    SmartDashboard.putNumber("yaxis_joy", yaxis);
    SmartDashboard.putNumber("zaxis_joy", zaxis);

    if(Math.abs(zaxis)<0.03){
      zaxis=0;
    }
    double err;
    err = Utils.navx.getRate() - mapAngRate(zaxis);
    double p_corr = 0.25*err;
    steer_corr += err*0.02;
    leftSpeed =  yaxis*RobotSettings.ysens + (-1*(steer_corr + p_corr));
    rightSpeed = yaxis*RobotSettings.ysens+ (steer_corr + p_corr);
    drive(leftSpeed, rightSpeed);
  }

//moveToAng
//-------------------------------------
  double i_corr_moveToAng = 0;
  public void moveToAng(double heading, double v)
  {
    double kp = 0.003;
    double ki = 0.000009;
    double error = heading - Utils.getCleanedHeading();
    i_corr_moveToAng += error*ki;
    double correction = i_corr_moveToAng + error*kp;
    correction = Math.signum(correction)*Math.min(Math.abs(correction), 0.1);
    drive(v+correction, v-correction);
  }
//-------------------------------------


//utility functions 
//------------------------
  public double mapAngRate(double z)
  {
    //Map z_axis to angular rate to a similia range of navx.getRate()
    return z*RobotSettings.zsens;
  }

  public void drive(double left, double right)
  {
    FR.set(right);
    BR.set(right);
    FL.set(left);
    BL.set(left);
  }

  double left_speed = 0, right_speed=0;

  double smoothDeadband = 0.02;
  public void smoothDrive(double left, double right)
  {
    left += left + (0.02 * RobotSettings.acceleration_max * Math.abs(left_speed-left)>smoothDeadband?(left_speed<left?1:-1) : 0);
    right += right + (0.02 * RobotSettings.acceleration_max * Math.abs(right_speed-right)>smoothDeadband?(right_speed<right?1:-1) : 0);
    FR.set(right);
    BR.set(right);
    FL.set(left);
    BL.set(left);
  }

//-------------------------


}
