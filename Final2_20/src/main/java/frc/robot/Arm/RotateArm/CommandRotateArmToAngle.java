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

public class CommandRotateArmToAngle extends Command {
 
  double encoderCountsPerDegrees = 11.11;
  double desired, maximum, value, velocity, error;
  

  public CommandRotateArmToAngle(double angle, double vel, double max) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(Robot.rotateArmSubsystem);

    desired = angle * encoderCountsPerDegrees;
    maximum = max;
    velocity = vel;


  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  double kP = 0.0;

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    value = Robot.rotateArmSubsystem.rotEnc.get(); 

    SmartDashboard.putNumber("Encoder get value", value);

    error = desired - value;

    if (velocity > maximum) { 
      velocity = maximum; 
    } else {
      velocity += error * kP;
    }

    Robot.rotateArmSubsystem.rotMotor.set(velocity);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(error) <= 5*encoderCountsPerDegrees);
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
