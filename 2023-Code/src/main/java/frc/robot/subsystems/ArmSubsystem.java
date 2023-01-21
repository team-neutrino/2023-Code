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

  private CANSparkMax m_armMotor = new CANSparkMax(Constants.MotorConstants.ARM_MOTOR1, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_armMotor.getEncoder();
  private SparkMaxPIDController m_pidController;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_armMotor.restoreFactoryDefaults();
    m_pidController = m_armMotor.getPIDController();
    m_encoder.setPositionConversionFactor(Constants.ArmConstants.ROTATION_TO_INCHES);
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

  public void setSpeed(double speed) {
    m_armMotor.set(speed);
  }

  public void setReference(double rotations) {
    // multiply angle by some constant
    m_pidController.setReference(rotations, ControlType.kPosition);
  }

  public double getSpeed() {
    return m_armMotor.get();
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  @Override
  public void periodic() {}
}
