// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class ClawSubsystem extends SubsystemBase {

  private CANSparkMax m_intakeMotor =
      new CANSparkMax(MotorConstants.INTAKEMOTORW, MotorType.kBrushless);


  public ClawSubsystem() {
    m_intakeMotor.restoreFactoryDefaults();
  }

  public void acquireCone() {
    m_intakeMotor.set(MotorConstants.INTAKEW_MOTOR_SPEED);
  }

  /** Runs the wheels motor in reverse. */
  public void spitOutCone() {
    m_intakeMotor.set(-MotorConstants.INTAKEW_MOTOR_SPEED);
  }

  /** Stops the motors. */
  public void stopMotor() {
    m_intakeMotor.set(0);
  }

  @Override
  public void periodic() {}
}
