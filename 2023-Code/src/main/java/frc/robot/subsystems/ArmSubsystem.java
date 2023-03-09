// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DigitalConstants;
import frc.robot.Constants.MotorConstants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax m_armMotor = new CANSparkMax(MotorConstants.ARM_MOTOR, MotorType.kBrushless);
  private CANSparkMax m_telescopingMotor = new CANSparkMax(MotorConstants.TELESCOPING_MOTOR, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_armMotor.getEncoder();
  private RelativeEncoder m_telescopingEncoder = m_telescopingMotor.getEncoder();
  private DutyCycleEncoder m_externalEncoder = new DutyCycleEncoder(DigitalConstants.ARM_ENCODER);
  private DutyCycleEncoder m_telescopingexternalEncoder = new DutyCycleEncoder(DigitalConstants.TELESCOPING_ENCODER);
  private SparkMaxPIDController m_pidController = m_armMotor.getPIDController();

  public ArmSubsystem() {
    m_armMotor.restoreFactoryDefaults();
    m_telescopingMotor.restoreFactoryDefaults();
    m_armMotor.setIdleMode(IdleMode.kBrake);
    m_telescopingMotor.setIdleMode(IdleMode.kBrake);
  }

  public double getAbsoluteArmPosition() {
    return m_externalEncoder.getAbsolutePosition() * 100;
  }

  public void turnArmOff() {
    m_armMotor.setVoltage(0);
  }

  public void setArmVoltage(double voltage) {
    m_armMotor.setVoltage(voltage);
  }

  public void setArm(double voltage) {
    m_armMotor.set(voltage);
  }

  public double limitArmAmount(double voltage) {
    if (voltage < -Constants.ArmConstants.ARM_OUTPUT_LIMIT) {
      voltage = -Constants.ArmConstants.ARM_OUTPUT_LIMIT;
    } else if (voltage > Constants.ArmConstants.ARM_OUTPUT_LIMIT) {
      voltage = Constants.ArmConstants.ARM_OUTPUT_LIMIT;
    }
    return voltage;
  }

  public void smartSet(double desiredVoltage) {
    if ((getAbsoluteArmPosition() >= ArmConstants.ARM_FRONTMOST && desiredVoltage > 0)
        || (getAbsoluteArmPosition() <= ArmConstants.ARM_BACKMOST && desiredVoltage < 0)) {
      setArm(0.0);
    } else {
      setArm(desiredVoltage);
    }
  }

  public void setReference(double rotations) {
    m_pidController.setReference(rotations, ControlType.kPosition);
  }

  public double getArmVoltage() {
    double appliedOutput = m_armMotor.getAppliedOutput();
    double busVoltage = m_armMotor.getBusVoltage();
    return appliedOutput * busVoltage; // chiefdelphi.com/t/get-voltage-from-spark-max/344136/3
  }

  public double getArmPosition() {
    return m_encoder.getPosition();
  }

  public boolean atArmPosition(double targetPosition) {
    if (Math.abs(getAbsoluteArmPosition() - targetPosition) < ArmConstants.ARM_DEADZONE) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {}
}
