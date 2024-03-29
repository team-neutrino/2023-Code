// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DigitalConstants;
import frc.robot.Constants.MotorConstants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax m_positionMotor =
      new CANSparkMax(MotorConstants.ARM_MOTOR, MotorType.kBrushless);

  private DutyCycleEncoder m_positionEncoder = new DutyCycleEncoder(DigitalConstants.ARM_ENCODER);

  public ArmSubsystem() {
    m_positionMotor.restoreFactoryDefaults();
    m_positionMotor.restoreFactoryDefaults();
    m_positionMotor.setIdleMode(IdleMode.kBrake);
    m_positionMotor.setIdleMode(IdleMode.kBrake);
  }

  public double getAbsoluteArmPosition() {
    return m_positionEncoder.getAbsolutePosition() * 100;
  }

  public void turnArmOff() {
    m_positionMotor.setVoltage(0);
  }

  public void setArmVoltage(double voltage) {
    m_positionMotor.setVoltage(voltage);
  }

  public void setArm(double voltage) {
    m_positionMotor.set(voltage);
  }

  public double limitArmAmount(double voltage) {
    if (voltage < -ArmConstants.ARM_OUTPUT_LIMIT) {
      voltage = -ArmConstants.ARM_OUTPUT_LIMIT;
    } else if (voltage > ArmConstants.ARM_OUTPUT_LIMIT) {
      voltage = ArmConstants.ARM_OUTPUT_LIMIT;
    }
    return voltage;
  }

  public void smartArmSet(double desiredVoltage) {
    if ((getAbsoluteArmPosition() >= ArmConstants.ARM_FRONTMOST && desiredVoltage > 0)
        || (getAbsoluteArmPosition() <= ArmConstants.ARM_BACKMOST && desiredVoltage < 0)) {
      setArm(0.0);
    } else {
      setArm(desiredVoltage);
    }
  }

  public double getArmVoltage() {
    double appliedOutput = m_positionMotor.getAppliedOutput();
    double busVoltage = m_positionMotor.getBusVoltage();
    return appliedOutput * busVoltage; // chiefdelphi.com/t/get-voltage-from-spark-max/344136/3
  }

  public DutyCycleEncoder getArmEncoder() {
    return m_positionEncoder;
  }

  public boolean atArmPosition(double targetPosition) {
    if (Math.abs(getAbsoluteArmPosition() - targetPosition) < ArmConstants.ARM_DEADZONE) {
      return true;
    }
    return false;
  }

  public static double encoderToRadians(double armEncoderTicks) {
    return -.02 * Math.PI * armEncoderTicks + 2.12 * Math.PI;
  }

  @Override
  public void periodic() {}
}
