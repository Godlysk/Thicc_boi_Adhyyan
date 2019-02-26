/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.buttons.*;
import frc.robot.Robot;
import frc.robot.Arm.RotateArm.*;

public class Joy2 {
    Button CommandRotateArmToAngleButton;
    public int grabberOpenButton = 1;
    public int grabberCloseButton = 2;
    public int grabberMoveDownButton =3;
    public int grabberMoveUpButton = 5;
    public int rotateArmToAngleButton = 4;
    public int robotLifterUpButton = 6;
    public int robotLifterDownButton = 7;
    public int robotLifterForwardButton = 8;
    public int robotLifterBackwardButton = 9;
    public int grabberShooterOpen = 11;
    public int grabberShooterClose =12;
    public Joy2() {
        // rotate arm to angle function - button 5
        CommandRotateArmToAngleButton = new JoystickButton(Robot.oi.joy2, rotateArmToAngleButton);
        CommandRotateArmToAngleButton.whileHeld(new CommandRotateArmToAngle(30));

    }
}
