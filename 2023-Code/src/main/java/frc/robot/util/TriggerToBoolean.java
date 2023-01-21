// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;

/**
 * A static only class that allows for the conversion of double trigger depression values to a
 * boolean, similar to the other buttons on the xbox controller. ex: if the left trigger is
 * depressed past .5 (halfway), return true.
 */
public class TriggerToBoolean {
  /** in place to prevent instantiation */
  private TriggerToBoolean() {}

  public static boolean leftTriggerPressed(XboxController controller) {
    return controller.getLeftTriggerAxis() >= .5;
  }

  public static boolean rightTriggerPressed(XboxController controller) {
    return controller.getRightTriggerAxis() >= .5;
  }
}
