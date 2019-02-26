/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Arm.RotateArm;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Encoder;

/**
 * Add your docs here.
 */
public class SubsystemRotateArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public WPI_VictorSPX rotMotor;
  public Encoder rotEnc;

  public final double meterPerPulse = 0.00001;

  public SubsystemRotateArm(){
    rotMotor = new WPI_VictorSPX (RobotMap.rotateArm_port);
    rotEnc = new Encoder(10, 11, false, Encoder.EncodingType.k4X);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CommandRotateArmJoy());
  }

  // public void moveToHeight(double height)  {
  //   double cur = encLift.get()*meterPerPulse - height;
  //   double speed = 
  // }
}
