/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSettings;
/**
 * An example command.  You can replace me with your own command.
 */
public class CommandRetardedDrive extends Command {
  public CommandRetardedDrive() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.tankDriveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.tankDriveSubsystem.drive(0,0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double yaxis = Robot.oi.getY(Robot.oi.joy1, 0.1);
    double zaxis = Robot.oi.getZ(Robot.oi.joy1, RobotSettings.zthresh);
    Robot.tankDriveSubsystem.PIDRetardedDrive(yaxis, zaxis);
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !Robot.oi.joy1.getRawButton(Robot.joystick1.retardedButton);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.tankDriveSubsystem.drive(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    
  }
}