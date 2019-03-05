/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.*;
import frc.robot.Utils;

public class CommandAutonomousDock extends Command {
  public CommandAutonomousDock() {
    requires(Robot.tankDriveSubsystem);
    requires(Robot.visionSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() { 
    Robot.tankDriveSubsystem.moveToAng(Robot.visionSubsystem.getTargFollowAng(), Robot.oi.getY(Robot.oi.joy1)*RobotSettings.autonomousDockSens);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !Robot.oi.joy1.getRawButton(7); //|| ((System.currentTimeMillis() - Robot.visionSubsystem.lastSeenTime) > 300) || Utils.getUltra()<40;//stops when you press 11 or when it stops seeing the target  }
  }
  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.tankDriveSubsystem.drive(0,0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
