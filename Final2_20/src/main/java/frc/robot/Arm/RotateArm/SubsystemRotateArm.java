/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Arm.RotateArm;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Utils;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;

/**
 * Add your docs here.
 */
public class SubsystemRotateArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public double i_corr;

  public WPI_TalonSRX rotMotor;
  public Encoder rotEnc;
  public AHRS armNavx;

  public final double meterPerPulse = 0.00001;

  public SubsystemRotateArm(){
    rotMotor = new WPI_TalonSRX (RobotMap.rotateArm_port);
    rotEnc = new Encoder(RobotMap.rotateArmEncoder, RobotMap.rotateArmEncoder+1, false, Encoder.EncodingType.k4X);
    armNavx = new AHRS(I2C.Port.kOnboard);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CommandRotateArmJoy());
  }

  public double getAngle(){
    return armNavx.getRoll() - Utils.navx.getPitch();
    //return Math.atan((-armNavx.getRawAccelY())/(-armNavx.getRawAccelY()));
  }
}
