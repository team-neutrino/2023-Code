// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Motor for the left wheels of the intake system */
  private CANSparkMax m_leftMotor = new CANSparkMax(
    Constants.MotorConstants.INTAKEMOTOR1,
    MotorType.kBrushless); // motor type subject to change;

  /** Another motor for the right wheels of the intake system. */
  private CANSparkMax m_rightMotor =
  new CANSparkMax(
      Constants.MotorConstants.INTAKEMOTOR2,
      MotorType.kBrushless); // motor type subject to change;

  /** Encoder for the left motor */
  private RelativeEncoder m_leftEncoder;

  /** Encoder for the right motor */
  private RelativeEncoder m_rightEncoder;

  /** Solenoid for the intake. */
  private Solenoid m_IntakeSolenoid =
  new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.INTAKE_PCM);

  /** Creates a new IntakeSubsystem and initializes the motor controllers. */
  public IntakeSubsystem() {
    m_rightMotor.restoreFactoryDefaults();
    m_leftMotor.restoreFactoryDefaults();
    //encoders initialized in constructor to make sure motors are initialized first
    m_leftEncoder = m_leftMotor.getEncoder();
    m_rightEncoder = m_rightMotor.getEncoder();
  }

  /**
   * Puts the intake down and runs it at a fixed speed.
   */
  public void runIntake() 
  {
    m_IntakeSolenoid.set(true);
    m_leftMotor.set(.2); // NEED TO MAKE CONSTANT FOR MOTOR SPEED
    m_rightMotor.set(.2);
  }

  /**
   * Sets the motors' inversion status to true.
   */
  public void reverseMotors()
  {
    m_leftMotor.setInverted(true);
    m_rightMotor.setInverted(true);
  }

  /**
   * Sets the motors' inversion status to false.
   */
  public void unreverseMotors()
  {
    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(false);
  }

  /**
   * Stops the motors and puts the whole thing up.
   */
  public void stopIntake()
  {
    m_leftMotor.set(0);
    m_rightMotor.set(0);
    m_IntakeSolenoid.set(false);
  }

  public double getRightEncoder()
  {
    return m_rightEncoder.getVelocity();
  }

  public double getLeftEncoder()
  {
    return m_leftEncoder.getVelocity();
  }

  /** Resets the endcoders */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
