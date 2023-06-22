// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Limiter;

public class MotorSubsystem extends SubsystemBase {

  Joystick m_joystick;

  public MotorSubsystem(Joystick p_joystick) {
    m_joystick = p_joystick;
  }

  // ** Make motors here **//
  CANSparkMax m_motor1 = new CANSparkMax(2, MotorType.kBrushless);

  @Override
  public void periodic() {
    // don't call both of these at the same time
    // m_motor1.set(0.1);
    m_motor1.set(Limiter.deadzone(m_joystick.getY(), 0.1));
  }
}
