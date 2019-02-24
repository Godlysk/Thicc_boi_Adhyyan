/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.RobotLifter;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class CommandRobotLifterUp extends Command {
  public CommandRobotLifterUp() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.lifterUpSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    boolean upButton = Robot.oi.joy2.getRawButton(Robot.oi.robotLifterUpButton);
    boolean downButton = Robot.oi.joy2.getRawButton(Robot.oi.robotLifterDownButton);

    if(upButton) {
      Robot.lifterUpSubsystem.lift(0.3); 
    }
    else if(downButton) {
      Robot.lifterUpSubsystem.lift(-0.3);
    }else{
      Robot.lifterUpSubsystem.lift(0);
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
