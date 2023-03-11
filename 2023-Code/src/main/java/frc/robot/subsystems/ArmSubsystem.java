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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DigitalConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.util.Bounder;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax m_armMotor = new CANSparkMax(MotorConstants.ARM_MOTOR, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_armMotor.getEncoder();
  private DutyCycleEncoder m_externalEncoder = new DutyCycleEncoder(DigitalConstants.ARM_ENCODER);
  private SparkMaxPIDController m_pidController = m_armMotor.getPIDController();

  public ArmSubsystem() {
    m_armMotor.restoreFactoryDefaults();
    m_armMotor.setIdleMode(IdleMode.kBrake);
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

  public double limitArmVoltage(double voltage) {
    return Bounder.bound(voltage, ArmConstants.ARM_OUTPUT_LIMIT);
  }

  public void smartSet(double desiredVoltage) {
    if ((getAbsolutePosition() >= ArmConstants.ARM_FRONTMOST && desiredVoltage > 0)
        || (getAbsolutePosition() <= ArmConstants.ARM_BACKMOST && desiredVoltage < 0)) {
      set(0.0);
    } else {
      set(desiredVoltage);
    }
  }

  public void setReference(double rotations) {
    m_pidController.setReference(rotations, ControlType.kPosition);
  }

  public double getVoltage() {
    double appliedOutput = m_armMotor.getAppliedOutput();
    double busVoltage = m_armMotor.getBusVoltage();
    return appliedOutput
        * busVoltage; // equation taken from chiefdelphi.com/t/get-voltage-from-spark-max/344136/3
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
