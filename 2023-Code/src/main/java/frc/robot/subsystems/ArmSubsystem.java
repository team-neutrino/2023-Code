// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax m_armMotor1 = new CANSparkMax(Constants.MotorConstants.ARM_MOTOR1, MotorType.kBrushless);;
  private CANSparkMax m_armMotor2 = new CANSparkMax(Constants.MotorConstants.ARM_MOTOR2, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_armMotor2.getEncoder();
  //private MotorControllerGroup m_motorGroup;

  private SparkMaxPIDController m_pidController;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_armMotor1.restoreFactoryDefaults();
    m_armMotor2.restoreFactoryDefaults();
    m_armMotor1.follow(m_armMotor2);
    //m_motorGroup = new MotorControllerGroup(m_armMotor1, m_armMotor2);
    m_pidController = m_armMotor2.getPIDController();
    //m_pidController = new SparkMaxPIDController(m_armMotor1);
    m_pidController.setFeedbackDevice(m_encoder);
    m_pidController.setP(Constants.PIDConstants.ARM_P);
    m_pidController.setI(Constants.PIDConstants.ARM_I);
    m_pidController.setD(Constants.PIDConstants.ARM_D);
    m_pidController.setFF(Constants.PIDConstants.ARM_FF);
    m_pidController.setOutputRange(
      Constants.PIDConstants.ARM_MINIMUM, 
      Constants.PIDConstants.ARM_MAXIMUM
    );
  }

  public void setSpeedArmMotor1(double speed) {
    m_armMotor1.set(speed);
  }

  public void setSpeedArmMotor2(double speed) {
    m_armMotor2.set(speed);
  }

  public double getSpeedArmMotor1() {
    return m_armMotor1.get();
  }

  public double getSpeedArmMotor2() {
    return m_armMotor2.get();
  }

  public void setReference(double rotations) {
    // multiply angle by some constant
    m_pidController.setReference(rotations, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
