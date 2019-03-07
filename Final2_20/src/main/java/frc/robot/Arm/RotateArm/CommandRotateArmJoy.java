 /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Arm.RotateArm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CommandRotateArmJoy extends Command {
  public CommandRotateArmJoy() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.rotateArmSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double yaxis = Robot.oi.getY(Robot.oi.joy2, 0) * 1;
    
    if(Math.abs(yaxis)<0.03){
      yaxis = 0;
    }else if(yaxis>=0.02 && yaxis<0.15){
      yaxis = 0.15;
    }
    else if(yaxis<-0.02 && yaxis>-0.15){
      yaxis = -0.15;
    }

    
    Robot.rotateArmSubsystem.rotMotor.set(-yaxis);
    
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.rotateArmSubsystem.rotMotor.set(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
