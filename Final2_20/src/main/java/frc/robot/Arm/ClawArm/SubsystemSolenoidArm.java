/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Arm.ClawArm;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.RobotMap;
import frc.robot.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;


/**
 * Add your docs here.
 */
public class SubsystemSolenoidArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public DoubleSolenoid ds;

  public SubsystemSolenoidArm(){
    ds = new DoubleSolenoid(0, 4);
    //rotEnc = new Encoder(, false, Encoder.EncodingType.k4X);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CommandSolenoidArm());
  }

  public void open(){
    ds.set(Value.kForward);
  }

  public void close(){
    ds.set(Value.kReverse);
  }
}
