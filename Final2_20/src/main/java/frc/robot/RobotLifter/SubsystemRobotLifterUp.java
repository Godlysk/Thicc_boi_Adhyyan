/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.RobotLifter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Utils;
import edu.wpi.first.wpilibj.*;

/**
 * Add your docs here.
 */
public class SubsystemRobotLifterUp extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  WPI_TalonSRX motor1;
  WPI_TalonSRX motor2;
  Encoder lifterUpEncoder1;
  Encoder lifterUpEncoder2;

  public SubsystemRobotLifterUp() {
    motor1 = new WPI_TalonSRX(RobotMap.robotLifterUpMotor1);
    motor2 = new WPI_TalonSRX(RobotMap.robotLifterUpMotor2);

    lifterUpEncoder1 = new Encoder(RobotMap.elevatorEncoder_l, (RobotMap.elevatorEncoder_l +1), true, Encoder.EncodingType.k4X);
    lifterUpEncoder2 = new Encoder(RobotMap.elevatorEncoder_r, (RobotMap.elevatorEncoder_r +1), true, Encoder.EncodingType.k4X);
  }

  public void lift(double lifterSpeed) {
    motor1.set(lifterSpeed);
    motor2.set(lifterSpeed);
  }

  public void liftingSpeeds(double front, double back) {
    motor1.set(front);
    motor2.set(back);
  }

  double enc_F, enc_B, derivative, integral_enc = 0;
  double pitch, xrate, proportional = 0;
  double integral_gyro = 0;
  double kP_gyro = 0.0;
  double kD_gyro = 0.0;
  double kI_encoder = 0.0;
  double kI_gyro = 0.0;

  public void PIDlifter(double f, double b) {
    enc_F = lifterUpEncoder1.getRate();
    enc_B = lifterUpEncoder2.getRate();
    pitch = Utils.navx.getPitch();
    xrate = Utils.navx.getRawGyroX();
    
    proportional = -pitch;
    integral_enc += (enc_F - enc_B);
    derivative = xrate;
    integral_gyro += -pitch;

    double correction = (proportional * kP_gyro) + (derivative * kD_gyro) + (integral_enc * kI_encoder) + (integral_gyro * kI_gyro); 
    
    f -= correction;
    b += correction;

    liftingSpeeds(f, b);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new CommandRobotLifterUpDown());
  }
}
