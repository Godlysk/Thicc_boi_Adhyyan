/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSettings;

public class CommandMoveToDistance extends Command {
  
  double circum = 15.24*Math.PI;
  double error, correction, targetspeed, currspeed;
  double rotations, degrees, travelled, desired_rotations, init, max_velocity,  acceleration_constant, desired_acceleration_period, loop_time;
  double kp;

  long last_time = 0;

  double error_leeway = 10;
  
  public CommandMoveToDistance(double max, double dist) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.tankDriveSubsystem);
    max_velocity = max;
    desired_rotations =  dist/circum;
    kp = Math.abs((1/(desired_rotations*360))*1.001);
    if(kp>0.05)
    {
      kp = 0.05;
    }

    desired_acceleration_period = 400; 
    loop_time = 20;

    acceleration_constant = max_velocity/(desired_acceleration_period/loop_time);
    currspeed = 0;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.tankDriveSubsystem.enc_r.reset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    

    // if (travelled<desired_rotations/2) {
    //   if (current_velocity < max_velocity) {
    //     current_velocity += acceleration_constant;
    //   } else {
    //     current_velocity = max_velocity;
    //   } 
    // } else {
    //   current_velocity -= acceleration_constant;
    // }

    // travelled = Robot.tankDriveSubsystem.enc_l.get() - init;
    
    error = desired_rotations*360 - ((Robot.tankDriveSubsystem.enc_r.get() / RobotSettings.USDigitalConstant)*360);
    
    correction = error * kp;

    if(correction>max_velocity){
      correction = max_velocity;
    }
    else if(correction<-max_velocity){
      correction = -max_velocity;
    }

    if(Math.abs(error) > error_leeway)
    {
      last_time = System.currentTimeMillis();
    }

    targetspeed = correction*(1/RobotSettings.ysens[0]);

    if (targetspeed > currspeed) {
      currspeed += acceleration_constant;
    } else {
      currspeed = targetspeed;
    }

    Robot.tankDriveSubsystem.PIDRetardedDrive(currspeed, 0);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(error)<error_leeway && System.currentTimeMillis() - last_time > 200;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.tankDriveSubsystem.drive(0,0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
  
}
