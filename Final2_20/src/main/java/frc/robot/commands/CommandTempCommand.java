/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSettings;

public class CommandTempCommand extends Command {
  public CommandTempCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.tankDriveSubsystem);
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
    Robot.visionSubsystem.getTarget();
    double error = Robot.visionSubsystem.targX - RobotSettings.center;
    double correction = error*0.008;
    double yaxis = Robot.oi.getY(Robot.oi.joy1, 0.07);

    Robot.tankDriveSubsystem.drive(yaxis*0.4 - correction, 0.4*yaxis + correction);
    
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
