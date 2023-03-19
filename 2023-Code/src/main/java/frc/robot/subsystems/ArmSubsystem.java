// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DigitalConstants;
import frc.robot.Constants.MotorConstants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax m_positionMotor =
      new CANSparkMax(MotorConstants.ARM_MOTOR, MotorType.kBrushless);
  private CANSparkMax m_telescopingMotor =
      new CANSparkMax(MotorConstants.TELESCOPING_MOTOR, MotorType.kBrushless);

  private DutyCycleEncoder m_positionEncoder = new DutyCycleEncoder(DigitalConstants.ARM_ENCODER);
  private Encoder m_telescopingEncoder =
      new Encoder(
          Constants.DigitalConstants.TELESCOPING_ENCODERA,
          Constants.DigitalConstants.TELESCOPING_ENCODERB);

  private SparkMaxLimitSwitch m_limitSwitch;

  public ArmSubsystem() {
    m_positionMotor.restoreFactoryDefaults();
    m_telescopingMotor.restoreFactoryDefaults();
    m_telescopingMotor.setSmartCurrentLimit(20);
    m_positionMotor.setIdleMode(IdleMode.kBrake);
    m_telescopingMotor.setIdleMode(IdleMode.kBrake);
    m_limitSwitch =
        m_telescopingMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    m_limitSwitch.enableLimitSwitch(true);
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
    if (voltage < -Constants.ArmConstants.ARM_OUTPUT_LIMIT) {
      voltage = -Constants.ArmConstants.ARM_OUTPUT_LIMIT;
    } else if (voltage > Constants.ArmConstants.ARM_OUTPUT_LIMIT) {
      voltage = Constants.ArmConstants.ARM_OUTPUT_LIMIT;
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

  public boolean atArmPosition(double targetPosition) {
    if (Math.abs(getAbsoluteArmPosition() - targetPosition) < ArmConstants.ARM_DEADZONE) {
      return true;
    }
    return false;
  }

  public double getDistance() {
    return m_telescopingEncoder.getDistance();
  }

  public void turnTelescopeOff() {
    m_telescopingMotor.setVoltage(0);
  }

  public void setTelescopeVoltage(double p_voltage) {
    m_telescopingMotor.setVoltage(p_voltage);
  }

  public void setTelescope(double p_output) {
    System.out.println(p_output);
    if (p_output < 0 || getEncoderDistance() > Constants.ArmConstants.TELESCOPE_EXTEND_LIMIT) {
      m_telescopingMotor.set(p_output);
    } else if (getEncoderDistance() <= Constants.ArmConstants.TELESCOPE_EXTEND_LIMIT) {
      m_telescopingMotor.set(0);
    }
  }

  public boolean getSwitch() {
    return false;
    // return m_limitSwitch.get();
  }

  public void setEncoder() {
    m_telescopingEncoder.setDistancePerPulse(1);
  }

  public double getEncoderDistance() {
    return m_telescopingEncoder.getDistance();
  }

  public void resetEncoder() {
    if (m_limitSwitch.isPressed()) {
      m_telescopingEncoder.reset();
    }
  }

  @Override
  public void periodic() {
    System.out.println(getDistance());
    resetEncoder();
  }
}
