// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PneumaticsConstants;

public class IntakeSubsystem extends SubsystemBase {

  /** Motor for the wheels of the intake system */
  private CANSparkMax m_wheelsMotor =
      new CANSparkMax(
          MotorConstants.INTAKEMOTOR, MotorType.kBrushless); // motor type subject to change;

  /** Encoder for the wheels on the intake */
  private RelativeEncoder m_wheelsEncoder;

  /** Encoder for the up-down motion of the intake */
  private RelativeEncoder m_upDownEncoder;

  /** Solenoid for the intake, controls the in-out motion */
  private Solenoid m_upDownSolenoid =
      new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.UP_DOWN_SOLENOID);

  private Solenoid m_squeezeSolenoid =
      new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.IN_OUT_SOLENOID);

  /** Beam break to detect if a game piece is present in the intake (to squeeeeze) */
  private DigitalInput m_beamBreak = new DigitalInput(DigitalConstants.INDEX_BEAMBREAK);

  /** Creates a new IntakeSubsystem and initializes the motor controllers. */
  public IntakeSubsystem() {
    m_wheelsMotor.restoreFactoryDefaults();
    // encoders initialized in constructor to make sure motors are initialized first
    m_wheelsEncoder = m_wheelsMotor.getEncoder();
  }

  /** Sets the solenoid to the 'out' position */
  public void setIntakeDown() {
    m_upDownSolenoid.set(true);
  }

  /** Sets the solenoid to the 'in' position. */
  public void setIntakeUp() {
    m_upDownSolenoid.set(false);
  }

  /** Runs the wheels motor at a fixed speed. */
  public void runIntake() {
    m_wheelsMotor.set(.4); // NEED TO MAKE CONSTANT FOR MOTOR SPEED
  }

  /** Runs the wheels motor in reverse. */
  public void runIntakeReverse() {
    m_wheelsMotor.set(-.4);
  }

  /** Stops the motors. */
  public void stopIntake() {
    m_wheelsMotor.set(0);
  }

  /**
   * Shows if a game piece is in the intake.
   *
   * @return True if there is an object blocking the beam, false otherwise.
   */
  public boolean getBeamBreak() {
    return m_beamBreak.get();
  }

  /**
   * Returns whether or not a game piece is in the intake.
   *
   * @return True if a game piece is present.
   */
  public boolean detectedGamePiece() {
    return !getBeamBreak();
  }

  /** Pushes the intake out a little */
  public void unsqueeze() {
    m_squeezeSolenoid.set(false);
  }

  /** Puts the intake back to its narrow form */
  public void squeeze() {
    m_squeezeSolenoid.set(true);
  }

  /**
   * Returns the state of the up-down solenoid
   *
   * @return whether or not the solenoid is in the out position.
   */
  public boolean isIntakeDown() {
    return m_upDownSolenoid.get();
  }

  /**
   * Returns the state of the SQEEZE solenoid
   *
   * @return Whether or not the solenoid is in the out position.
   */
  public boolean getSqueezeSolenoidValue() {
    return m_squeezeSolenoid.get();
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
  public void periodic() {}
}
