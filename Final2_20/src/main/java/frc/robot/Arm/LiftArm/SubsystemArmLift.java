/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Arm.LiftArm;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.RobotMap;
import frc.robot.*;


/**
 * Add your docs here.
 */
public class SubsystemArmLift extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public WPI_VictorSPX liftMotor;
  public Encoder encLift;

  public final double meterPerPulse = 0.00001;

  public SubsystemArmLift(){
    liftMotor = new WPI_VictorSPX (RobotMap.liftMotor_port);
    encLift = new Encoder(RobotMap.liftArmEncoder, (RobotMap.liftArmEncoder+1), false, Encoder.EncodingType.k4X);
  }

  @Override
  public void initDefaultCommand() {
    
    setDefaultCommand(new CommandMoveLift());
  }


  // public void moveToHeight(double height)  {
  //   double cur = encLift.get()*meterPerPulse - height;
  //   double speed = 
  // }

}
