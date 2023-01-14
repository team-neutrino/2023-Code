// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Motor for the intake system */
  private CANSparkMax m_motor1;

  /** Another motor for the intake system. */
  private CANSparkMax m_motor2;

  /** ANOTHER motor for the intake system. */
  private CANSparkMax m_motor3;

  /** Encoder for motor1 */
  private RelativeEncoder m_encoder1;

  /** Encoder for motor2 */
  private RelativeEncoder m_encoder2;

  /** encoder for motor3 */
  private RelativeEncoder m_encoder3;

  /** Solenoid for the intake. */
  private Solenoid m_IntakeSolenoid;

  /** A group of all intake subsystem motors. */
  MotorControllerGroup m_motors;

  /** Creates a new IntakeSubsystem and initializes the motor controllers. */
  public IntakeSubsystem() {
    m_motor1.restoreFactoryDefaults();
    m_motor2.restoreFactoryDefaults();
    m_motor3.restoreFactoryDefaults();

    m_encoder1 = m_motor1.getEncoder();
    m_encoder2 = m_motor2.getEncoder();
    m_encoder3 = m_motor3.getEncoder();

    m_motor1.setIdleMode(IdleMode.kBrake);
    m_motor2.setIdleMode(IdleMode.kBrake);
    m_motor3.setIdleMode(IdleMode.kBrake);

    m_motor1 =
        new CANSparkMax(
            Constants.MotorConstants.INTAKEMOTOR1,
            MotorType.kBrushless); // motor type subject to change
    m_motor2 =
        new CANSparkMax(
            Constants.MotorConstants.INTAKEMOTOR2,
            MotorType.kBrushless); // motor type subject to change
    m_motor3 =
        new CANSparkMax(
            Constants.MotorConstants.INTAKEMOTOR3,
            MotorType.kBrushless); // motor type subject to change

    m_IntakeSolenoid =
        new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.INTAKE_PCM);

    m_motors = new MotorControllerGroup(m_motor1, m_motor2, m_motor3);
  }

  /**
   * Sets the speed of the intake motors.
   *
   * @param p_motorSpeed The speed to set the motors to.
   */
  public void runIntake() {
    m_IntakeSolenoid.toggle();
    m_motors.set(.2); // NEED TO MAKE CONSTANT FOR MOTOR SPEED
  }

  /** Stops all motors. */
  public void stopMotors() {
    m_motors.set(0);
  }

  /** Resets the endcoders */
  public void resetEncoders() {
    m_encoder1.setPosition(0);
    m_encoder3.setPosition(0);
    m_encoder3.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
