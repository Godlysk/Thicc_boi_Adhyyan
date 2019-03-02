/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.buttons.*;
import frc.robot.CommandGroupHatchPanelCollect;

public class Joy2 {
    Button CommandRotateArmToAngleButton, CommandGroupHatchPanelCollectButton;
    public int grabberOpenButton = 2;
    public int grabberCloseButton = 1;
    public int grabberLiftDownButton =3;
    public int grabberLiftUpButton = 5;
    public int hatchPanelCollectButton = 4;
    public int robotLifterUpButton = 8;
    public int robotLifterDownButton = 7;
    
    public int robotLifterForwardButton = 10;
    public int robotLifterBackwardButton = 9;

    public int grabberShooterOpen = 11;
    public int grabberShooterClose = 12;

    
    public Joy2() {
        // rotate arm to angle function - button 4
        // CommandRotateArmToAngleButton = new JoystickButton(Robot.oi.joy2, rotateArmToAngleButton);
        // CommandRotateArmToAngleButton.whileHeld(new CommandRotateArmToAngle(30));

        CommandGroupHatchPanelCollectButton = new JoystickButton(Robot.oi.joy2, hatchPanelCollectButton);
        CommandGroupHatchPanelCollectButton.whenReleased(new CommandGroupHatchPanelCollect());
    }
}
