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
import frc.robot.Constants.TelescopeConstants;

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

  public void setTelescope(double p_output, double armPos) {
    if (p_output > 0
        && !canExtend(armPos)) { // if we're trying to extend but we shouldn't, don't and retract
      retractTelescoping();
    } else if ((p_output < 0
        || getTelescopingExtension() > TelescopeConstants.TELESCOPE_EXTEND_LIMIT)) {
      m_telescopingMotor.set(p_output);
    } else {
      m_telescopingMotor.set(0);
    }
  }

  public void retractTelescoping() {
    setTelescope(TelescopeConstants.TELESCOPE_RETRACT_SPEED, -1);
  }

  public void extendTelescoping(double armPos) {
    setTelescope(TelescopeConstants.TELESCOPE_EXTEND_SPEED, armPos);
  }

  /**
   * Helper method that tells whether we can safely extend the telescoping arm or not.
   *
   * @param armPos The current angle of the arm as given by getAbsoluteArmPosition.
   * @return Whether or not the arm can safely extend.
   */
  private boolean canExtend(double armPos) {
    if ((armPos < ArmConstants.FORWARD_ARM_HEIGHT_LIMIT
            && armPos > ArmConstants.BACKWARD_ARM_HEIGHT_LIMIT)
        || armPos > 84) {
      return false;
    }
    return true;
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
  }
}
