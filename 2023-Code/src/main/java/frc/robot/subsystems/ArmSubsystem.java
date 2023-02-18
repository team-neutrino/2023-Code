// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DigitalConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PIDConstants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax m_armMotor = new CANSparkMax(MotorConstants.ARM_MOTOR, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_armMotor.getEncoder();
  private DutyCycleEncoder m_externalEncoder = new DutyCycleEncoder(DigitalConstants.ARM_ENCODER);
  private SparkMaxPIDController m_pidController;

  public ArmSubsystem() {
    m_armMotor.restoreFactoryDefaults();

    m_pidController = m_armMotor.getPIDController();
    m_encoder.setPositionConversionFactor(ArmConstants.ROTATION_TO_INCHES);

    m_pidController.setFeedbackDevice(m_encoder);
    m_pidController.setP(PIDConstants.ARM_P);
    m_pidController.setI(PIDConstants.ARM_I);
    m_pidController.setD(PIDConstants.ARM_D);
    m_pidController.setFF(PIDConstants.ARM_FF);
    m_pidController.setOutputRange(PIDConstants.ARM_BACKMOST, PIDConstants.ARM_FRONTMOST);
  }

  public double getAbsolutePosition() {
    return m_externalEncoder.getAbsolutePosition() * 100;
  }

  public void turnOff() {
    m_armMotor.setVoltage(0);
  }

  public void setVoltage(double voltage) {
    m_armMotor.setVoltage(voltage);
  }

  public void set(double voltage) {
    m_armMotor.set(voltage);
  }

  public void setReference(double rotations) {
    m_pidController.setReference(rotations, ControlType.kPosition);
  }

  public double getVoltage() {
    double appliedOutput = m_armMotor.getAppliedOutput();
    double busVoltage = m_armMotor.getBusVoltage();
    return appliedOutput * busVoltage; // chiefdelphi.com/t/get-voltage-from-spark-max/344136/3
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  public boolean atPosition(double targetPosition) {
    if (Math.abs(getAbsolutePosition() - targetPosition) < ArmConstants.ARM_DEADZONE) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {}
}
