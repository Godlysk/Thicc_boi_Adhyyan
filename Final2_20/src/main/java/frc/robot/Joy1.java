/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import frc.robot.Robot;
import frc.robot.Chassis.CommandRetardedDrive;
import frc.robot.commands.CommandTrackServo;
import frc.robot.Chassis.CommandBrake;

public class Joy1 {
    
    Button commandRetardedDriveButton, commandBrakeMoveButton, commandTrackServoButton;
    public int steerButton = 2;
    public int brakeButton = 1;
    public int expButton = 10;
    public int frontRobotLifterUpButton = 5;
    public int frontRobotLifterDownButton = 3;
    public int backRobotLifterUpButton = 6;
    public int backRobotLifterDownButton = 4;
    
    
    public int servoTrackButton = 8;
    

    public Joy1() {
        //Steer drive function - button 1
        commandRetardedDriveButton = new JoystickButton(Robot.oi.joy1, steerButton);
        commandRetardedDriveButton.whileHeld(new CommandRetardedDrive());

        //Command Brake - Chassis - stops everything in the chassis subsystem
        commandBrakeMoveButton = new JoystickButton(Robot.oi.joy1, brakeButton);
        commandBrakeMoveButton.whileHeld(new CommandBrake());


        //Temporary Command track servo, delete later
        commandTrackServoButton = new JoystickButton(Robot.oi.joy1, servoTrackButton);
        commandTrackServoButton.whileHeld(new CommandTrackServo());
    }

}