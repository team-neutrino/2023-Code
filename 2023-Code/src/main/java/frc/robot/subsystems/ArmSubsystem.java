// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax m_armMotor =
      new CANSparkMax(Constants.MotorConstants.ARM_MOTOR1, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_armMotor.getEncoder();
  private DutyCycleEncoder m_externalEncoder =
      new DutyCycleEncoder(Constants.DigitalConstants.ARM_ENCODER);
  private SparkMaxPIDController m_pidController;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_armMotor.restoreFactoryDefaults();
    setSoftLimits();

    m_pidController = m_armMotor.getPIDController();
    m_encoder.setPositionConversionFactor(Constants.ArmConstants.ROTATION_TO_INCHES);

    m_pidController.setFeedbackDevice(m_encoder);
    m_pidController.setP(Constants.PIDConstants.ARM_P);
    m_pidController.setI(Constants.PIDConstants.ARM_I);
    m_pidController.setD(Constants.PIDConstants.ARM_D);
    m_pidController.setFF(Constants.PIDConstants.ARM_FF);
    m_pidController.setOutputRange(
        Constants.PIDConstants.ARM_MINIMUM, Constants.PIDConstants.ARM_MAXIMUM);
  }

  public double getAbsolutePosition() {
    return m_externalEncoder.getAbsolutePosition() * 100;
  }

  private void setSoftLimits() {
    m_armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_armMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.ArmConstants.MAX_SOFT_LIM);
    m_armMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.ArmConstants.MIN_SOFT_LIM);
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
    if (Math.abs(getAbsolutePosition() - targetPosition) < Constants.ArmConstants.ARM_DEADZONE) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // System.out.println(getAbsolutePosition());
    
  }
}
