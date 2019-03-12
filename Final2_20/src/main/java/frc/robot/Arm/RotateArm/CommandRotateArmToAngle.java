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
  }

  double kP = 0.01;

  // Called repeatedly when this Command is scheduled to run

  

  @Override
  protected void execute() {

    curAngle = Robot.rotateArmSubsystem.rotEnc.get()/encoderCountsPerDegrees;

    SmartDashboard.putNumber("Rotate Arm Angle", curAngle);

    error = desiredAngle - curAngle;
    rotateVelocity = error * kP;
    rotateVelocity = Utils.inAbsRange(rotateVelocity, 0.1, maximum);

    SmartDashboard.putNumber("Arm Rotate Velocity", rotateVelocity);
    Robot.rotateArmSubsystem.rotMotor.set(rotateVelocity);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(error) <= 5);
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
