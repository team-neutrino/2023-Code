// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.util.EnumConstants.LEDColor;

public class MagicButton extends CommandBase {

  private DriveTrainSubsystem m_drivetrainSubsystem;
  private ArmSubsystem m_armSubsystem;
  private LEDSubsystem m_ledSubsystem;
  private boolean isShift;

  private int[][] lesliesBS = new array[(1,1,1), ]

  /*
   *     (angle)          cone | isShift
   *      ifForward   
   *                  ifForward | cone (adds)
   *      ifShift         
   * 
   *      0,1 ifForward
   *      0,1 ifShift
   *      0,1 Cone 
   *      
   *      0,1,2 = 
   * 
   * output: (angle and extension)
   * 
   * FIRST VALUE
   * forward index 0 + cone index 1, 
   * back no-shift index 2 + cone index 1, 
   * back shift index 3 + cone 1, 
   * 
   * firstVal
   * if(forward):
   *    firstValue += 0
   * if(back):
   *    firstValue += 2
   *    if(shift):
   *        firstValue+=2
   * if(cone):
   *    firstValue+=1
   * 
   * [(forward-mid-cube), (forward-mid-cones), 
   * (back-mid-cube, (back-mid-cone), 
   * back-high-cube), back-high-cone)
   * ]
   * 
  
  if(backward):
    forwardBack = 1
  if(high):
    midHigh = 1
    extension = true
  if(cone):
    cubeCone = 1
  
   theArray[forwardBack][midHigh][cubeCone]
   * [
      [ #forward
        [ # mid
          a, # cube
          b # cone
        ], 
        [ # high
          a', # cube
          b' # cone
        ]
      ], 
      [ #back
        [ # mid
          c, # cube
          d # cone
        ], 
        [ # high
          c', # cube
          d' # cone
        ]
      ]
     ]
   *
   * 
  
   * 
   * m_armSubsystem.setArm([FIRST VALUE])
   * m_armSubsystem.sextExtension[ifShift)]
   * 
   * 
   *  if(isForward):
   *      forward = 1;
   * else:
   *      forward = 0
   *  
   *  if(isShift):
   *    shift = 1
   *  else:
   *    shift = 0
   * 
   * if(cone):
   *    cone = 1
   * else: 
   *    cone = 0
   * 
   * dictionary: key-000 to 111 and value is a method
   */   


  public MagicButton(DriveTrainSubsystem p_drivetrainSubsystem, ArmSubsystem p_armSubsystem, LEDSubsystem p_ledSubsystem) {
    m_drivetrainSubsystem = p_drivetrainSubsystem;
    m_armSubsystem = p_armSubsystem;
    m_ledSubsystem = p_ledSubsystem;
  }
  
  /*
    * - forward or back: based on encoder
    *    - cone or cube: based on LED
    *    - extension: based on SHIFT (start)
    *    - angle: XBOX is the magic button
    *        - forward + shift + cone = mid++
    *        - forward + shift + cube = mid
    *        - forward + noShift + cone = hybrid
    *        - forward + noShift + cube = hybrid
    *        - backward + shift + cone = high++
            - backward + shift + cube = high
    *
            - backward + noShift + cone = mid++
            - backward + noShift + cube = mid
   */

  @Override
  public void initialize() {}


  @Override
  public void execute() {
    boolean isForward = m_drivetrainSubsystem.getYaw() > -90 && m_drivetrainSubsystem.getYaw() < 90;
    boolean isShfit = m_armSubsystem.getIsShift();
    boolean isCone = m_ledSubsystem.getColor() == LEDColor.YELLOW;
    if (isForward) {
      if (isShift) {
        if (isCone) {

        }
        else {
          
        }
      } 
      else {
        if (isCone) {

        }
        else {
          
        }
      }
    }
    else {
      if (isShift) {
        if (isCone) {

        }
        else {
          
        }
      }
      else {
        if (isCone) {

        }
        else {
          
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
