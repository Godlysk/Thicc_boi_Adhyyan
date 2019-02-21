/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.CommandRetardedDrive;
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


  public Encoder enc_r = new Encoder(0, 1,true, Encoder.EncodingType.k4X);
  public Encoder enc_l = new Encoder(2, 3,false, Encoder.EncodingType.k4X);

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
    setDefaultCommand(new CommandRetardedDrive());
  }

//SteerDrive
//---------------------------------
  double steer_corr = 0;
  public void steerDrive()
  {
    double yaxis = Robot.oi.getY(Robot.oi.joy1);
    double zaxis = Robot.oi.getZ(Robot.oi.joy1);
    double err, leftSpeed, rightSpeed;
    err = Utils.navx.getRate() - mapAngRate(zaxis);
    double p_corr = 0.1*err;
    steer_corr += err*0.01;
    leftSpeed =  yaxis*RobotSettings.ysens + (-1*(steer_corr + p_corr));
    rightSpeed = yaxis*RobotSettings.ysens+ (steer_corr + p_corr);

    drive(leftSpeed, rightSpeed);
  }
//---------------------------------
  

//PIDRetardedDrive
//-------------------------------------
  
  double d_gain = 0.3;
  double i_gain = 0.0001;
  double rightSpeed , leftSpeed;
  double s_d_corr, s_i_corr = 0;
  double t_corr=0;

  public void PIDRetardedDrive(){
    double yaxis = Robot.oi.getY(Robot.oi.joy1);
    double zaxis = Robot.oi.getZ(Robot.oi.joy1);

    double err;  
    if(Math.abs(yaxis) > Math.abs(zaxis)){
      err = enc_l.getRate() - enc_r.getRate();
      s_d_corr = Utils.navx.getRate() * d_gain;
      s_i_corr += err*i_gain;

      rightSpeed = yaxis*0.5 + s_d_corr + s_i_corr;
      leftSpeed = yaxis*0.5 - s_d_corr - s_i_corr;

    }else if(Math.abs(yaxis)<Math.abs(zaxis) && Math.abs(zaxis)>RobotSettings.zthresh){
      err = Utils.navx.getRate() - mapAngRate(zaxis);
      double p_corr = 0.1*err;
      t_corr+=err*0.01;

      leftSpeed =  -1*(t_corr + p_corr);
      rightSpeed = (t_corr + p_corr);
    }
    else
    {
      t_corr = 0;
      s_d_corr = 0;
      s_i_corr = 0;
      rightSpeed = 0;
      leftSpeed = 0;
    }
    drive(leftSpeed, rightSpeed);
  }
//-------------------------------------

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

//-------------------------


}
