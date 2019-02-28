/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Arm.ClawArm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CommandArmWheels extends Command {
  public CommandArmWheels() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.armShooterWheels);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    boolean shooterForward = Robot.oi.joy2.getRawButton(Robot.joystick2.grabberShooterOpen);
    boolean shooterBackward = Robot.oi.joy2.getRawButton(Robot.joystick2.grabberShooterClose);

    if(shooterForward) {
      Robot.armShooterWheels.turn(0.6);
    }else if(shooterBackward) {
      Robot.armShooterWheels.turn(-0.6);
    }else {
      Robot.armShooterWheels.turn(0);
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
