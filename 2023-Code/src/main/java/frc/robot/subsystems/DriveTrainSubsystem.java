// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSubsystem extends SubsystemBase {

  Joystick m_joystick;

  public DriveTrainSubsystem(Joystick p_joystick)
  {
    m_joystick = p_joystick;
  }

  //** Make motors here **//
  CANSparkMax m_motor1 = new CANSparkMax(1, MotorType.kBrushless);


  @Override
  public void periodic()
  {
    //don't call both of these at the same time
    m_motor1.set(0.1);
    m_motor1.set(m_joystick.getY());
  }  
}