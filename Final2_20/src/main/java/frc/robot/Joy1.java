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

public class Joy1 {
    Button commandSteerDriveButton;
    public static int steerButton = 1;
    public static int expButton = 10;

    public Joy1() {
        //Steer drive function - button 1
        commandSteerDriveButton = new JoystickButton(Robot.oi.joy1, steerButton);
        commandSteerDriveButton.whileHeld(new CommandSteerDrive());
    }

}