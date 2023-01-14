// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmManualSubsystem extends SubsystemBase {
  private CANSparkMax m_armMotor1 = new CANSparkMax(100, MotorType.kBrushless); 
  private RelativeEncoder m_encoder1 = m_armMotor1.getEncoder();
  private double m_initialAngle;
  private double m_armAngle;
  

  /** Creates a new ArmManualSubsystem. */
  public ArmManualSubsystem() {
    m_initialAngle = m_encoder1.getPosition();
  }

  public void setSetpointAngle(double setpointAngle) {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_armAngle = m_encoder1.getPosition() - m_initialAngle;
  }
}
