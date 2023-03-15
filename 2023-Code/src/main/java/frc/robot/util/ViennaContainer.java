// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;
import frc.robot.util.ViennaPIDController;
import frc.robot.Constants.PIDConstants;


public class ViennaContainer {
    public static ViennaPIDController getArmSetPositionController() {
        return new ViennaPIDController(PIDConstants.ARM_P, PIDConstants.ARM_I, PIDConstants.ARM_D);
    }
    public static ViennaPIDController getArmAdjustController(){
      return new ViennaPIDController(PIDConstants.ARM_P_ADJUST, PIDConstants.ARM_I, PIDConstants.ARM_D);
    }
}