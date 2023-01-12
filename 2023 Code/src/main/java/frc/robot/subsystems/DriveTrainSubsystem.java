// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class DriveTrainSubsystem extends SubsystemBase {

  private CANSparkMax m_rmotor1 = new CANSparkMax(Constants.MotorConstants.RMOTOR1, MotorType.kBrushless);
  private CANSparkMax m_rmotor2 = new CANSparkMax(Constants.MotorConstants.RMOTOR2, MotorType.kBrushless);
  private CANSparkMax m_lmotor1 = new CANSparkMax(Constants.MotorConstants.LMOTOR1, MotorType.kBrushless);
  private CANSparkMax m_lmotor2 = new CANSparkMax(Constants.MotorConstants.LMOTOR2, MotorType.kBrushless);

  MotorControllerGroup m_rmotors = new MotorControllerGroup(m_rmotor1, m_rmotor2);
  MotorControllerGroup m_lmotors = new MotorControllerGroup(m_lmotor1, m_lmotor2);

  /** Creates a new Drivetrain. */
  public DriveTrainSubsystem() {

  m_rmotor1.restoreFactoryDefaults();
  m_rmotor2.restoreFactoryDefaults();
  m_lmotor1.restoreFactoryDefaults();
  m_rmotor2.restoreFactoryDefaults();


  m_rmotor1.setInverted(true);
  m_rmotor2.setInverted(true);
  m_lmotor1.setInverted(false);
  m_lmotor2.setInverted(false);
  

  }

  public void setMotors(double m_rightMotorSpeed, double m_leftMotorSpeed)
  {
    m_rmotors.set(m_rightMotorSpeed);
    m_lmotors.set(m_leftMotorSpeed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
