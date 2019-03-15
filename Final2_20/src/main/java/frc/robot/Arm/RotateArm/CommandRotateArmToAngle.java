/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Arm.RotateArm;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Utils;

public class CommandRotateArmToAngle extends Command {
 
  double encoderCountsPerDegrees = 11.11;
  double desiredAngle, maximum, curAngle, error, rotateVelocity;
  double deadband = 1.5;

  public CommandRotateArmToAngle(double angle, double max) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.rotateArmSubsystem);

    desiredAngle = angle;
    maximum = max;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.rotateArmSubsystem.i_corr = 0;
  }

  double kp = 0.03;

  // Called repeatedly when this Command is scheduled to run

  @Override
  protected void execute() {

    curAngle = Robot.rotateArmSubsystem.getAngle();

    //SmartDashboard.putNumber("Rotate Arm Angle", curAngle);


    error = desiredAngle - curAngle;
    Robot.rotateArmSubsystem.i_corr += error * 0.0001;
    rotateVelocity = error * kp + Robot.rotateArmSubsystem.i_corr;
    rotateVelocity = Utils.inAbsRange(rotateVelocity, 0.15, maximum);

    //SmartDashboard.putNumber("Arm Rotate Velocity", rotateVelocity);
    if(!isInDeadband()){
      Robot.rotateArmSubsystem.rotMotor.set(rotateVelocity);
    }else{
      Robot.rotateArmSubsystem.i_corr = 0;
    }
  }


  public boolean isInDeadband()
  {
    return (Math.abs(error) <= deadband);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isInDeadband();
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
