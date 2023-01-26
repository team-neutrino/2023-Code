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

  /** Motor for the wheels of the intake system */
  private CANSparkMax m_wheelsMotor =
      new CANSparkMax(
          Constants.MotorConstants.INTAKEMOTOR1,
          MotorType.kBrushless); // motor type subject to change;

  /** Another motor for the setting up and putting down of the intake. */
  private CANSparkMax m_upDownMotor =
      new CANSparkMax(
          Constants.MotorConstants.INTAKEMOTOR2,
          MotorType.kBrushless); // motor type subject to change;

  /** Encoder for the wheels on the intake */
  private RelativeEncoder m_wheelsEncoder;

  /** Encoder for the up-down motion of the intake */
  private RelativeEncoder m_upDownEncoder;

  /** Solenoid for the intake, controls the in-out motion */
  private Solenoid m_IntakeSolenoid =
      new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.INTAKE_PCM);

  /** Creates a new IntakeSubsystem and initializes the motor controllers. */
  public IntakeSubsystem() {
    m_wheelsMotor.restoreFactoryDefaults();
    m_upDownMotor.restoreFactoryDefaults();
    // encoders initialized in constructor to make sure motors are initialized first
    m_upDownEncoder = m_upDownMotor.getEncoder();
    m_wheelsEncoder = m_wheelsMotor.getEncoder();
  }

  /** Sets the solenoid to the 'out' position */
  public void setIntakeDown() {
    m_IntakeSolenoid.set(true);
  }

  /** Sets the solenoid to the 'in' position. */
  public void setIntakeUp() {
    m_IntakeSolenoid.set(false);
  }

  /** Runs the wheels motor at a fixed speed. */
  public void runIntake() {
    m_wheelsMotor.set(.2); // NEED TO MAKE CONSTANT FOR MOTOR SPEED
  }

  /** Runs the wheels motor in reverse. */
  public void runIntakeReverse() {
    m_wheelsMotor.set(-.2);
  }

  /** Stops the motors. */
  public void stopIntake() {
    m_wheelsMotor.set(0);
  }

  public double getWheelsEncoder() {
    return m_wheelsEncoder.getVelocity();
  }

  public double getUpDownEncoder() {
    return m_upDownEncoder.getVelocity();
  }

  /** Resets the endcoders */
  public void resetEncoders() {
    m_upDownEncoder.setPosition(0);
    m_wheelsEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
