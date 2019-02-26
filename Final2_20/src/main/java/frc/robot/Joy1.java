/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.buttons.*;
import frc.robot.Chassis.*;
import frc.robot.Robot;

public class Joy1 {
    
    Button commandSteerDriveButton, commandBrakeMoveButton;
    public int steerButton = 1;
    public int brakeButton = 2;
    public int expButton = 10;
    public int frontRobotLifterUpButton = 4;
    public int frontRobotLifterDownButton =5;
    public int backRobotLifterUpButton =6 ;
    public int backRobotLifterDownButton = 7;
    

    public Joy1() {
        //Steer drive function - button 1
        commandSteerDriveButton = new JoystickButton(Robot.oi.joy1, steerButton);
        commandSteerDriveButton.whileHeld(new CommandSteerDrive());

        commandBrakeMoveButton = new JoystickButton(Robot.oi.joy1, brakeButton);
        commandBrakeMoveButton.whileHeld(new CommandBrake());
    }

}