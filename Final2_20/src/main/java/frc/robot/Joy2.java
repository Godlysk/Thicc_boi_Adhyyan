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

public class Joy2 {
    Button CommandRotateArmToAngleButton;
    public static int grabberOpenButton = 1;
    public static int grabberCloseButton = 2;
    public static int grabberMoveDownButton =3;
    public static int grabberMoveUpButton =5;
    public static int rotateArmToAngleButton = 4;
    public static int robotLifterUpButton = 6;
    public static int robotLifterDownButton = 7;
    public static int robotLifterForwardButton = 8;
    public static int robotLifterBackwardButton = 9;
    public Joy2() {
        // rotate arm to angle function - button 5
        CommandRotateArmToAngleButton = new JoystickButton(Robot.oi.joy2, rotateArmToAngleButton);
        CommandRotateArmToAngleButton.whileHeld(new CommandRotateArmToAngle(30));
    }
}