/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*--------------------------------------this--------------------------------------*/

package frc.robot.Arm.RotateArm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CommandRotateArmToAngle extends Command {
  double targetAngle;
  //4000 divided by 360
  double encoderCountsPerDegrees = 11.11;
  public CommandRotateArmToAngle(double desiredAngle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.rotateArmSubsystem);
    targetAngle = desiredAngle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double actual = Robot.rotateArmSubsystem.rotEnc.get()/encoderCountsPerDegrees;
    double error = targetAngle - actual;
    double turningConstant = 0.03;
    double turningSpeed = error * turningConstant; 
    System.out.println("ERROR: " + error);


    if(turningSpeed >= 0.6) {
      turningSpeed = 0.6;
    }else if(turningSpeed <= -0.6){
      turningSpeed = -0.6;
    }
    System.out.println("Speed: " + turningSpeed); 
    Robot.rotateArmSubsystem.rotMotor.set(turningSpeed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double error = targetAngle - Robot.rotateArmSubsystem.rotEnc.get(); 
    return (Math.abs(error) <= 4);
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
