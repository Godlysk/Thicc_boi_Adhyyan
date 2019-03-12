/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.buttons.*;
import frc.robot.Arm.RotateArm.CommandRotateArmByTime;
import frc.robot.Arm.RotateArm.CommandRotateArmToAngle;

public class Joy2 {
    Button CommandRotateArmToAngleButton, CommandRotateArmByTimeButton, CommandGroupHatchPanelCollectButton;
    public int grabberOpenButton = 1;
    public int grabberCloseButton = 2;
    public int grabberLiftUpButton = 3;
    public int grabberLiftDownButton =5;
    // public int hatchPanelCollectButton = 4;
    public int grabberRotateByTimeButton =4;
    public int robotLifterUpButton = 8;
    public int robotLifterDownButton = 7;

    public int rotateArmToAngleButton = 6;

    public int robotLifterForwardButton = 20;//10;
    public int robotLifterBackwardButton = 21;// 9;

    public int grabberShooterOpen = 11;
    public int grabberShooterClose = 12;

    
    public Joy2() { 
        CommandRotateArmToAngleButton = new JoystickButton(Robot.oi.joy2, rotateArmToAngleButton);
        CommandRotateArmToAngleButton.whileHeld(new CommandRotateArmToAngle(30, 0.4));

        // CommandGroupHatchPanelCollectButton = new JoystickButton(Robot.oi.joy2, hatchPanelCollectButton);
        // CommandGroupHatchPanelCollectButton.whenReleased(new CommandGroupHatchPanelCollect());

        CommandRotateArmByTimeButton = new JoystickButton(Robot.oi.joy2, grabberRotateByTimeButton);
        CommandRotateArmByTimeButton.whileHeld(new CommandRotateArmByTime(0.5, 0.5));

    }
}
