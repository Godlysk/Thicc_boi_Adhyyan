/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*--------------------------------------this--------------------------------------*/

package frc.robot.Arm.RotateArm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CommandRotateArmByAngle extends Command {
  
  
  
  double targetAngle;
  double error;

  double max;

  double offset;
  public CommandRotateArmByAngle(double desiredAngle, double maxSpeed) {

    requires(Robot.rotateArmSubsystem);
    targetAngle = desiredAngle;
    offset= 0;
    max = maxSpeed;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    offset = Robot.rotateArmSubsystem.getAngle();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double actual = Robot.rotateArmSubsystem.getAngle() - offset;
    error = targetAngle - actual;
    double turningConstant = 0.03;
    double turningSpeed = error * turningConstant; 
    if(turningSpeed >= max) {
      turningSpeed = max;
    }else if(turningSpeed <= -max){
      turningSpeed = -max;
    }
    Robot.rotateArmSubsystem.rotMotor.set(turningSpeed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(error) <= 3);
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
