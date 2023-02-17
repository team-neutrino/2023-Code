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
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax m_armMotor =
      new CANSparkMax(Constants.MotorConstants.ARM_MOTOR1, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_armMotor.getEncoder();
  private DutyCycleEncoder m_externalEncoder =
      new DutyCycleEncoder(Constants.DigitalConstants.ARM_ENCODER);
  private SparkMaxPIDController m_pidController = m_armMotor.getPIDController();
  private boolean softLimitOn = false;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    setSoftLimits(true);
    m_armMotor.restoreFactoryDefaults();
    m_encoder.setPositionConversionFactor(Constants.ArmConstants.ROTATION_TO_INCHES);
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

  public void setSoftLimits(boolean set) {
    softLimitOn = set;
  }

  public void smartSet(double desiredVoltage) {
    if (softLimitOn) {
      if ((getAbsolutePosition() >= ArmConstants.ARM_MAXIMUM && desiredVoltage < 0)
          || (getAbsolutePosition() <= ArmConstants.ARM_MINIMUM && desiredVoltage > 0)) {
        set(0.0);
      }
    }
    set(desiredVoltage);
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
  public void periodic() {}
}
