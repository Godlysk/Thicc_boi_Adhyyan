/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class CommandMoveServoJoy extends Command {
  public CommandMoveServoJoy() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.visionSubsystem);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  double angle = 90;
  @Override
  protected void execute() {
    
    if(Robot.oi.joy1.getPOV() == 270){
      angle = 180;
    }else if(Robot.oi.joy1.getPOV() == 90){
      angle = 0;
    }else if(Robot.oi.joy1.getPOV() == 0){
      angle = 90;
    }
    Robot.visionSubsystem.camServo.setAngle(angle);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
