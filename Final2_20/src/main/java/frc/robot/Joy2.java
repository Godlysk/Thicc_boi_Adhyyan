/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.buttons.*;
import frc.robot.Arm.RotateArm.CommandRotateArmByTime;
import frc.robot.Arm.RotateArm.CommandRotateArmToAngle;

public class Joy2 {
    Button CommandRotateArmToAngleButton, CommandRotateArmByTimeButton, CommandGroupHatchPanelCollectButton;
    Button CommandHatchHeightButton, CommandCargoCollectionHeightButton, CommandCargoRocketHeightbutton, CommandCargoShipHeightButton; 
    public int grabberOpenButton = 1;
    public int grabberCloseButton = 2;
    public int grabberLiftUpButton = 8;
    public int grabberLiftDownButton =7;
    // public int hatchPanelCollectButton = 4;
    //public int grabberRotateByTimeButton =4;
    public int robotLifterUpButton = 13;
    public int robotLifterDownButton = 14;

    public int hatchHeightButton = 5;
    public int cargoCollectionHieghtButton = 3;
    public int cargoRocketHeightbutton = 4;
    public int cargoShipHeightButton = 6;

    public int overrideArm = 9;


    public int robotLifterForwardButton = 20;//10;
    public int robotLifterBackwardButton = 21;// 9;

    public int grabberShooterOpen = 12;
    public int grabberShooterClose = 11;

    
    //arm navx values
    public double armAngleOffset = 0;
    public double HatchHeight = -38 + armAngleOffset;
    public double CargoCollectionHeight = 9 + armAngleOffset;
    public double CargoRocketHeight = -25 + armAngleOffset;
    public double CargoShipHeight = 13 + armAngleOffset;
    public double armVel = 0.4;

    public Joy2() { 
        CommandHatchHeightButton = new JoystickButton(Robot.oi.joy2, hatchHeightButton);
        CommandHatchHeightButton.whileHeld(new CommandRotateArmToAngle(HatchHeight, armVel));

        CommandCargoCollectionHeightButton = new JoystickButton(Robot.oi.joy2, cargoCollectionHieghtButton );
        CommandCargoCollectionHeightButton.whileHeld(new CommandRotateArmToAngle(CargoCollectionHeight, armVel));

        CommandCargoRocketHeightbutton = new JoystickButton(Robot.oi.joy2, cargoRocketHeightbutton);
        CommandCargoRocketHeightbutton.whileHeld(new CommandRotateArmToAngle(CargoRocketHeight, armVel));

        CommandCargoShipHeightButton = new JoystickButton(Robot.oi.joy2, cargoShipHeightButton );
        CommandCargoShipHeightButton.whileHeld(new CommandRotateArmToAngle(CargoShipHeight, armVel));
    }
}
