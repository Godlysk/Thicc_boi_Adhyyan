/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.buttons.*;
import frc.robot.Robot;
import frc.robot.Chassis.CommandRetardedDrive;
import frc.robot.commands.CommandAutonomousDock;
import frc.robot.commands.CommandTempCommand;
import frc.robot.commands.CommandTrackServo;
import frc.robot.Chassis.CommandBrake;

public class Joy1 {
    
    Button commandRetardedDriveButton, commandBrakeMoveButton, commandTrackServoButton;
    Button commandMoveSetDistanceButton, commandAutonomousDockButton, commandAutonomousDockSlanted;
    Button tempButton;
    public int retardedButton = 9;
    public int brakeButton = 1;
    //public int expButton = 10;
    public int frontRobotLifterUpButton = 5;
    public int frontRobotLifterDownButton = 3;
    public int backRobotLifterUpButton = 6;
    public int backRobotLifterDownButton = 4;
    public int navxReset = 12;
    

    public int moveSetDistance = 11;
    
    
    public int servoTrackButton = 10;
    public int autonomousDockButton = 7;
    public int autonomousDockSlantButton = 8; 
    

    public Joy1() {
        //retarded drive function - button 1
        commandRetardedDriveButton = new JoystickButton(Robot.oi.joy1, retardedButton);
        commandRetardedDriveButton.whileHeld(new CommandRetardedDrive());

        //Command Brake - Chassis - stops everything in the chassis subsystem
        commandBrakeMoveButton = new JoystickButton(Robot.oi.joy1, brakeButton);
        commandBrakeMoveButton.whileHeld(new CommandBrake());


        //Temporary Command track servo, delete later
        commandTrackServoButton = new JoystickButton(Robot.oi.joy1, servoTrackButton);
        commandTrackServoButton.whileHeld(new CommandTrackServo());

        // commandMoveSetDistanceButton= new JoystickButton(Robot.oi.joy1, moveSetDistance);
        // commandMoveSetDistanceButton.whenPressed(new CommandMoveToDistance(0.3, -200));

        commandAutonomousDockButton= new JoystickButton(Robot.oi.joy1, autonomousDockButton);
        commandAutonomousDockButton.whileHeld(new CommandAutonomousDock(false));

        commandAutonomousDockSlanted = new JoystickButton(Robot.oi.joy1, autonomousDockSlantButton);
        commandAutonomousDockSlanted.whileHeld(new CommandAutonomousDock(true));


        tempButton = new JoystickButton(Robot.oi.joy1, 11);
        tempButton.whileHeld(new CommandTempCommand());
        
    }

}