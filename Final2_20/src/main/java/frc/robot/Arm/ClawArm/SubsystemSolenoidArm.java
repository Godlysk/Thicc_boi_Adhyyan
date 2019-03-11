/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Arm.ClawArm;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;


/**
 * Add your docs here.
 */
public class SubsystemSolenoidArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public DoubleSolenoid ds;

  public SubsystemSolenoidArm(){
    ds = new DoubleSolenoid(RobotMap.pistonPortFirst, RobotMap.pistonPortSecond);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CommandSolenoidArm());
  }

  public void open(){
    ds.set(Value.kReverse);
  }

  public void close(){
    ds.set(Value.kForward);
  }
}
