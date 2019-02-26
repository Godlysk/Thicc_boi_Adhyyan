/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.RobotLifter;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CommandRobotLifterForward extends Command {
  public CommandRobotLifterForward() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.lifterForwardSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    Boolean forwardButton = Robot.oi.joy2.getRawButton(Robot.joystick2.robotLifterForwardButton);
    Boolean backwardButton = Robot.oi.joy2.getRawButton(Robot.joystick2.robotLifterBackwardButton);
    if(forwardButton) {
      Robot.lifterForwardSubsystem.drive(0.5);
    }else if(backwardButton) {
      Robot.lifterForwardSubsystem.drive(-0.5);
    }else {
      Robot.lifterForwardSubsystem.drive(0);
    }
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
