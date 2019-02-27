/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.RobotLifter;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CommandRobotLifterUpDown extends Command {
  public CommandRobotLifterUpDown() {
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
    boolean upButton = Robot.oi.joy2.getRawButton( Robot.joystick2.robotLifterUpButton);
    boolean downButton = Robot.oi.joy2.getRawButton( Robot.joystick2.robotLifterDownButton);

    //individual lifter mechanism buttons
    boolean leftUpButton = Robot.oi.joy1.getRawButton(Robot.joystick1.frontRobotLifterUpButton);    
    boolean leftDownButton = Robot.oi.joy1.getRawButton(Robot.joystick1.frontRobotLifterDownButton);
    boolean rightUpButton = Robot.oi.joy1.getRawButton(Robot.joystick1.backRobotLifterUpButton);
    boolean rightDownButton = Robot.oi.joy1.getRawButton(Robot.joystick1.backRobotLifterDownButton);

    //together PID
    if(upButton) {
      Robot.lifterUpSubsystem.lift(0.8); 
    }
    else if(downButton) {
      Robot.lifterUpSubsystem.lift(-0.8);
    }else{
      Robot.lifterUpSubsystem.lift(0);
    }

    //individual movement -up & down
    if(leftUpButton) {
      Robot.lifterUpSubsystem.liftingSpeeds(0.8, 0);
    }else if(leftDownButton) {
      Robot.lifterUpSubsystem.liftingSpeeds(-0.8, 0);
    }else if(rightUpButton) {
      Robot.lifterUpSubsystem.liftingSpeeds(0,0.8);
    }else if(rightDownButton) {
      Robot.lifterUpSubsystem.liftingSpeeds(0, -0.8);
    }else {
      Robot.lifterUpSubsystem.liftingSpeeds(0, 0);
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
