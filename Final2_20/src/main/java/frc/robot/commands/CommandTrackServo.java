/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CommandTrackServo extends Command {
  public CommandTrackServo() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.visionSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.visionSubsystem.camServo.setAngle(90);
  }


  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Robot.visionSubsystem.trackServo();
    //System.out.println("LMAO");
    Robot.visionSubsystem.camServo.setAngle(90);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !Robot.oi.joy1.getRawButton(Robot.joystick1.servoTrackButton);
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
