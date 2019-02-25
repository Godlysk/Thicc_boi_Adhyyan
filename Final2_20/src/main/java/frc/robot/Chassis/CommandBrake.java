/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Chassis;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class CommandBrake extends InstantCommand {
  /**
   * Add your docs here.
   */
  public CommandBrake() {
    super();
    // Use requires() here to declare subsystem dependencies
    requires(Robot.tankDriveSubsystem);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.tankDriveSubsystem.drive(0,0);
  }

}
