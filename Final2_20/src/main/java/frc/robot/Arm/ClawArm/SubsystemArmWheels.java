/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Arm.ClawArm;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class SubsystemArmWheels extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_TalonSRX shooterMotor1;
  public WPI_TalonSRX shooterMotor2;
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new CommandArmWheels());
  }

  public SubsystemArmWheels() {
    shooterMotor1 = new WPI_TalonSRX(RobotMap.shooterMotor1);
    shooterMotor2 = new WPI_TalonSRX(RobotMap.shooterMotor2);
    shooterMotor2.setInverted(true);
  }

  public void turn(double rotateSpeed) {
    shooterMotor1.set(rotateSpeed);
    shooterMotor2.set(rotateSpeed);
  }
}
