/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Arm.LiftArm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;

public class CommandMoveLift extends Command {
  public CommandMoveLift() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.armLiftSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    boolean moveUpButton = Robot.oi.joy2.getRawButton(Robot.joystick2.grabberMoveUpButton);
    boolean moveDownButton = Robot.oi.joy2.getRawButton(Robot.joystick2.grabberMoveDownButton);

    if(moveUpButton){
      Robot.armLiftSubsystem.liftMotor.set(0.8);
    }else if(moveDownButton){
      Robot.armLiftSubsystem.liftMotor.set(-0.7);
    }else{
      Robot.armLiftSubsystem.liftMotor.set(0);
    }

    // double yaxis = Robot.oi.joy2.getRawAxis(5);
    // Robot.armLiftSubsystem.liftMotor.set(yaxis*1);
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
