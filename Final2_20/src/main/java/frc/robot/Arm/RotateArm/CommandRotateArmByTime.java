/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Arm.RotateArm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CommandRotateArmByTime extends Command {

  Timer t;
  double maxVel, rotateForTime, rotateVelocity, rotationConstant = 0.8;
  public CommandRotateArmByTime(double max, double t) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.rotateArmSubsystem);
    maxVel = max;
    rotateForTime = t;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    t.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double timePassed = t.get();
    double timeLeft = rotateForTime - timePassed;

    double rotateVelocity = timeLeft*rotationConstant;
    double sign = Math.signum(rotateVelocity);
    if(Math.abs(rotateVelocity) > maxVel) {rotateVelocity = maxVel;}
    if(Math.abs(rotateVelocity) <0.1) {rotateVelocity =0.1;}
    rotateVelocity*= sign;
    Robot.rotateArmSubsystem.rotMotor.set(rotateVelocity);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double timeLeft = rotateForTime - t.get();
    return(timeLeft <= 0.15);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    t.stop();
    Robot.rotateArmSubsystem.rotMotor.set(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
