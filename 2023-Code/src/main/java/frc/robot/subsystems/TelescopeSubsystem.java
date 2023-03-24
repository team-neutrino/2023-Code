// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DigitalConstants;
import frc.robot.Constants.MotorConstants;

public class TelescopeSubsystem extends SubsystemBase {
  private CANSparkMax m_telescopingMotor =
      new CANSparkMax(MotorConstants.TELESCOPING_MOTOR, MotorType.kBrushless);

  private Encoder m_telescopingEncoder =
      new Encoder(DigitalConstants.TELESCOPING_ENCODERA, DigitalConstants.TELESCOPING_ENCODERB);

  private SparkMaxLimitSwitch m_limitSwitch;

  public TelescopeSubsystem() {
    m_telescopingMotor.setIdleMode(IdleMode.kBrake);

    m_telescopingMotor.restoreFactoryDefaults();
    m_telescopingMotor.setSmartCurrentLimit(22);
    m_limitSwitch =
        m_telescopingMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    m_limitSwitch.enableLimitSwitch(true);
    setEncoder();
  }

  public double getTelescopingVoltage() {
    double appliedOutput = m_telescopingMotor.getAppliedOutput();
    double busVoltage = m_telescopingMotor.getBusVoltage();
    return appliedOutput * busVoltage; // chiefdelphi.com/t/get-voltage-from-spark-max/344136/3
  }

  public void turnTelescopeOff() {
    m_telescopingMotor.setVoltage(0);
  }

  public void setTelescopeVoltage(double p_voltage) {
    m_telescopingMotor.setVoltage(p_voltage);
  }

  public void setTelescope(double p_output) {
    if ((p_output < 0 || getTelescopingExtension() > ArmConstants.TELESCOPE_EXTEND_LIMIT)) {
      m_telescopingMotor.set(p_output);
      // System.out.println("output===========" + p_output);
    } else {
      m_telescopingMotor.set(0);
      // System.out.println("=============output===========");
    }
  }

  public void retractTelescoping() {
    setTelescope(ArmConstants.TELESCOPE_RETRACT_SPEED);
  }

  public void extendTelescoping() {
    setTelescope(ArmConstants.TELESCOPE_EXTEND_SPEED);
  }

  public boolean isPressed() {
    return m_limitSwitch.isPressed();
  }

  public void setEncoder() {
    m_telescopingEncoder.setDistancePerPulse(1);
  }

  public double getTelescopingExtension() {
    return m_telescopingEncoder.getDistance();
  }

  public void resetEncoder() {
    if (m_limitSwitch.isPressed()) {
      m_telescopingEncoder.reset();
    }
  }

  @Override
  public void periodic() {
    resetEncoder();
    // System.out.println(getTelescopingExtension());
    // This method will be called once per scheduler run
  }
}
