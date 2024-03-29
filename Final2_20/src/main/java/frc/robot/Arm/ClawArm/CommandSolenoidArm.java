/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Arm.ClawArm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CommandSolenoidArm extends Command {

  boolean currentState;

  public CommandSolenoidArm() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.solenoidArmSubsystem);
    currentState = false;
  }

  // Called just before this Command runs the first time

  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    nonToggle();
  }


  public void toggle(){
    boolean toggleOpenCloseButton = Robot.oi.joy2.getRawButtonPressed(Robot.joystick2.grabberOpenButton);
    if(toggleOpenCloseButton){
      currentState = !currentState;
    }

    if(currentState){
      Robot.solenoidArmSubsystem.open();
      //set on
    }else{
      Robot.solenoidArmSubsystem.close();
      //set off
    }
  }

  public void nonToggle(){

    boolean open = Robot.oi.joy2.getRawButtonPressed(Robot.joystick2.grabberOpenButton);
    boolean close = Robot.oi.joy2.getRawButtonPressed(Robot.joystick2.grabberCloseButton);

    if(open){
      Robot.solenoidArmSubsystem.open();
    }else if(close){
      Robot.solenoidArmSubsystem.close();
    }else{
      //dont do anything
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
