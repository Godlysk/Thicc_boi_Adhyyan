/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CommandMoveToDistance extends Command {
  
  double circum = 15.24*Math.PI;
  double rotations, degrees, travelled, desired, init, max_velocity,  acceleration_constant = 0.001;
  double current_velocity = 0.0;
  
  public CommandMoveToDistance(double max, double dist) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.tankDriveSubsystem);
    max_velocity = max;
    desired = ((dist*60) / (Math.PI));

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    init = Robot.tankDriveSubsystem.enc_l.get();
    desired -= init;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (travelled < desired - travelled) {
      if (current_velocity < max_velocity) {
        current_velocity += acceleration_constant;
      } else {
        current_velocity = max_velocity;
      } 
    } else {
      current_velocity -= acceleration_constant;
    }

    travelled = Robot.tankDriveSubsystem.enc_l.get() - init;
    
    PID(current_velocity, current_velocity);

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

  double integral, error = 0;
  double kP = 0.0002;
  double kI = 0.00002;

  public void PID(double left, double right) {
    double speedL = Robot.tankDriveSubsystem.enc_l.getRate();
    double speedR = Robot.tankDriveSubsystem.enc_r.getRate();

    error = speedR - speedL;  
    integral += error;

    double correction = (error * kP) + (integral * kI);
    left -= correction;
    right += correction;
    Robot.tankDriveSubsystem.drive(left, right);
  }
  
}
