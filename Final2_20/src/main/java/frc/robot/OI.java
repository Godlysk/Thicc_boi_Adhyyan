/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import frc.robot.Chassis.*;
import frc.robot.RobotLifter.*;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Arm.RotateArm.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  Button commandSteerDriveButton, CommandRotateArmToAngleButton, CommandRobotLifterUp, CommandRobotLifterForward;



  public Joystick joy1, joy2;
  public static int steerButton = 1;
  public static int rotateArmToAngleButton = 4;
  public static int robotLifterUpButton = 6;
  public static int robotLifterDownButton = 7;
  public static int robotLifterForwardButton = 8;
  public static int robotLifterBackwardButton = 9;
  public static int expButton = 10;
  

  public OI(){
    joy1 = new Joystick(1);
    joy2 = new Joystick(2);
    //Steer drive function - button 1
    commandSteerDriveButton = new JoystickButton(joy1, steerButton);
    commandSteerDriveButton.whileHeld(new CommandSteerDrive());

    //rotate arm to angle function - button 5
    // CommandRotateArmToAngleButton = new JoystickButton(joy2, rotateArmToAngleButton);
    // CommandRotateArmToAngleButton.whileHeld(new CommandRotateArmToAngle(30));

  }

  public double getY(Joystick joy){
    return -joy.getY();
  }

  public double getZ(Joystick joy){
    return joy.getZ();
  }

  public double getX(Joystick joy){
    return joy.getX();
  }

  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
