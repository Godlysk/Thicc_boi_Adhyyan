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
    double yaxis = Robot.oi.getY(Robot.oi.joy2, 0) * 0.7;
    boolean negativeBoost = Robot.oi.joy2.getRawButton(Robot.joystick2.armRotateNegativeBoostButton);
    if(negativeBoost){
      yaxis *= 0.4;
    }
    
    //double upperbound = 0.07, lowerbound = 0.04;
    // if(Math.abs(yaxis)<lowerbound){
    //   yaxis = 0;
    // }
    // else if(yaxis>=lowerbound && yaxis<upperbound){
    //   yaxis = upperbound;
    // }
    // else if(yaxis<-lowerbound && yaxis>-upperbound){
    //   yaxis = -upperbound;
    // }

    
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
