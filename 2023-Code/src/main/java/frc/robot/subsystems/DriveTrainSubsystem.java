// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {

  private CANSparkMax m_rmotor1 =
      new CANSparkMax(Constants.MotorConstants.RMOTOR1, MotorType.kBrushless);
  private CANSparkMax m_rmotor2 =
      new CANSparkMax(Constants.MotorConstants.RMOTOR2, MotorType.kBrushless);
  private CANSparkMax m_rmotor3 =
      new CANSparkMax(Constants.MotorConstants.RMOTOR3, MotorType.kBrushless);
  private CANSparkMax m_lmotor1 =
      new CANSparkMax(Constants.MotorConstants.LMOTOR1, MotorType.kBrushless);
  private CANSparkMax m_lmotor2 =
      new CANSparkMax(Constants.MotorConstants.LMOTOR2, MotorType.kBrushless);
  private CANSparkMax m_lmotor3 =
      new CANSparkMax(Constants.MotorConstants.LMOTOR3, MotorType.kBrushless);

  private RelativeEncoder m_encoderR1;
  private RelativeEncoder m_encoderR2;
  private RelativeEncoder m_encoderR3;
  private RelativeEncoder m_encoderL1;
  private RelativeEncoder m_encoderL2;
  private RelativeEncoder m_encoderL3;

  double DEADZONE = 0.1;
  MotorControllerGroup m_rmotors = new MotorControllerGroup(m_rmotor1, m_rmotor2, m_rmotor3);
  MotorControllerGroup m_lmotors = new MotorControllerGroup(m_lmotor1, m_lmotor2, m_lmotor3);

  /** Creates a new Drivetrain. */
  public DriveTrainSubsystem() {

    m_rmotor1.restoreFactoryDefaults();
    m_rmotor2.restoreFactoryDefaults();
    m_rmotor3.restoreFactoryDefaults();
    m_lmotor1.restoreFactoryDefaults();
    m_rmotor2.restoreFactoryDefaults();
    m_lmotor3.restoreFactoryDefaults();

    m_rmotor1.setInverted(true);
    m_rmotor2.setInverted(true);
    m_rmotor3.setInverted(true);
    m_lmotor1.setInverted(false);
    m_lmotor2.setInverted(false);
    m_lmotor3.setInverted(true);

    m_encoderR1 = m_rmotor1.getEncoder();
    m_encoderR2 = m_rmotor2.getEncoder();
    m_encoderR3 = m_rmotor3.getEncoder();
    m_encoderL1 = m_lmotor1.getEncoder();
    m_encoderL2 = m_lmotor2.getEncoder();
    m_encoderL3 = m_lmotor3.getEncoder();

  }

  public double getEncoderPos(RelativeEncoder encoder){
    return encoder.getPosition();
  }

  public double getEncoderVel(RelativeEncoder encoder){
    return encoder.getVelocity();
  }

  public void setMotors(double m_rightMotorSpeed, double m_leftMotorSpeed) {
    m_leftMotorSpeed = linearAccel(deadzone(m_leftMotorSpeed));
    m_rightMotorSpeed = linearAccel(deadzone(m_rightMotorSpeed));
    m_rmotors.set(m_rightMotorSpeed);
    m_lmotors.set(m_leftMotorSpeed);
  }

  public double deadzone(double joystickY){
    if(joystickY<=DEADZONE){
      return 0.0;
    } else if(joystickY>=1.0){
      return 1.0;
    } else{
      return joystickY;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static double linearAccel(double joystickY){
    double newSpeed = joystickY;
    return newSpeed;
  }

  public static double slowAccel(double joystickY){
    double MAXSPEED = 0.7;
    double newSpeed = (2*MAXSPEED*joystickY) / (1 + Math.abs(joystickY));
    return newSpeed;
  }

  public static double turboAccel(double joystickY){
    double newSpeed = Math.pow(joystickY, 3)*1.6 + (0.17*joystickY);
    return newSpeed;
  }
}
