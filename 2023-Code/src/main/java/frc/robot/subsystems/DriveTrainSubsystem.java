// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {

  double cycle = 0;
  private AHRS m_navX = new AHRS(SPI.Port.kMXP);

  private CANSparkMax m_rmotor1 =
      new CANSparkMax(Constants.MotorConstants.RMOTOR1, MotorType.kBrushless);
  private CANSparkMax m_rmotor2 =
      new CANSparkMax(Constants.MotorConstants.RMOTOR2, MotorType.kBrushless);
  private CANSparkMax m_lmotor1 =
      new CANSparkMax(Constants.MotorConstants.LMOTOR1, MotorType.kBrushless);
  private CANSparkMax m_lmotor2 =
      new CANSparkMax(Constants.MotorConstants.LMOTOR2, MotorType.kBrushless);

  private RelativeEncoder m_encoderR1;
  private RelativeEncoder m_encoderR2;
  private RelativeEncoder m_encoderL1;
  private RelativeEncoder m_encoderL2;

  MotorControllerGroup m_rmotors = new MotorControllerGroup(m_rmotor1, m_rmotor2);
  MotorControllerGroup m_lmotors = new MotorControllerGroup(m_lmotor1, m_lmotor2);

  /** Creates a new Drivetrain. */
  public DriveTrainSubsystem() {

    m_rmotor1.restoreFactoryDefaults();
    m_rmotor2.restoreFactoryDefaults();
    m_lmotor1.restoreFactoryDefaults();
    m_rmotor2.restoreFactoryDefaults();

    m_rmotor1.setInverted(true);
    m_rmotor2.setInverted(true);
    m_lmotor1.setInverted(false);
    m_lmotor2.setInverted(false);

    m_encoderR1 = m_rmotor1.getEncoder();
    m_encoderR2 = m_rmotor2.getEncoder();
    m_encoderL1 = m_lmotor1.getEncoder();
    m_encoderL2 = m_lmotor2.getEncoder();

    m_encoderR1.setPosition(0);
    m_encoderR2.setPosition(0);
    m_encoderL1.setPosition(0);
    m_encoderL2.setPosition(0);
  }

  public double getR1Pos() {
    return m_encoderR1.getPosition();
  }

  public double getR2Pos() {
    return m_encoderR2.getPosition();
  }

  public double getL1Pos() {
    return m_encoderL1.getPosition();
  }

  public double getL2Pos() {
    return m_encoderL2.getPosition();
  }

  public double getR1Vel() {
    return m_encoderR1.getVelocity();
  }

  public double getR2Vel() {
    return m_encoderR2.getVelocity();
  }

  public double getL1Vel() {
    return m_encoderL1.getVelocity();
  }

  public double getL2Vel() {
    return m_encoderL2.getVelocity();
  }

  public void setMotors(double rightMotorInput, double leftMotorInput) {
    double leftMotorSpeed = linearAccel(deadzone(leftMotorInput));
    double rightMotorSpeed = linearAccel(deadzone(rightMotorInput));
    m_rmotors.set(rightMotorSpeed);
    m_lmotors.set(leftMotorSpeed);
  }

  public void setMotorsStraight(double m_bothMotorSpeed, double placeholder) {
    m_rmotors.set(m_bothMotorSpeed * -1);
    m_lmotors.set(m_bothMotorSpeed * 1);
  }

  public double deadzone(double joystickY) {
    double absJoystickY = Math.abs(joystickY);
    if (absJoystickY <= Constants.VariableConstants.DEADZONE) {
      return 0.0;
    } else if (absJoystickY >= 0.8) {
      return 1.0;
    } else {
      return joystickY;
    }
  }

  public void printPosition() {
    if (cycle % 20 == 0) {
      System.out.println("Positions are " + getL1Pos() + " " + getR1Pos());
    }
  }

  public void setMotorPosition(double rmotorset, double lmotorset) {
    rmotorset = rmotorset * 15;
    lmotorset = lmotorset * 15;
    double rmotorPosition = getR1Pos();
    double lmotorPosition = getL1Pos();

    while (getR1Pos() < rmotorPosition + rmotorset && getL1Pos() < lmotorPosition + lmotorset) {
      m_rmotors.set(0.3);
      m_lmotors.set(0.3);
    }

    m_rmotors.set(0);
    m_lmotors.set(0);
  }

  public void turnMotor(double motorset) {
    if (motorset > 0) {
      turnMotorRight(Math.abs(motorset));
      System.out.println("turning motor right");
    } else if (motorset < 0) {
      turnMotorLeft(Math.abs(motorset));
      System.out.println("turning motor left");
    }
  }

  public void turnMotorRight(double motorset) {
    motorset = motorset * 20 / Math.PI;
    double rmotorPosition = getR1Pos();
    double lmotorPosition = getL1Pos();

    while (getR1Pos() > rmotorPosition - motorset && getL1Pos() < lmotorPosition + motorset) {
      m_rmotors.set(-0.05);
      m_lmotors.set(0.05);
    }

    m_rmotors.set(0);
    m_lmotors.set(0);
  }

  public void turnMotorLeft(double motorset) {
    motorset = motorset * 20 / Math.PI;
    double rmotorPosition = getR1Pos();
    double lmotorPosition = getL1Pos();

    while (getR1Pos() < rmotorPosition + motorset && getL1Pos() > lmotorPosition - motorset) {
      m_rmotors.set(0.05);
      m_lmotors.set(-0.05);
    }

    m_rmotors.set(0);
    m_lmotors.set(0);
  }

  public double getPitch() {
    return m_navX.getPitch();
  }

  public void setVoltage(double voltage) {
    m_rmotors.setVoltage(voltage);
    m_lmotors.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    cycle++;
    printPosition();
    System.out.println(getPitch());
  }

  public static double linearAccel(double joystickY) {
    double newSpeed = joystickY;
    return newSpeed;
  }

  public static double slowAccel(double joystickY) {
    double MAXSPEED = 0.7;
    double newSpeed = (2 * MAXSPEED * joystickY) / (1 + Math.abs(joystickY));
    return newSpeed;
  }

  public static double turboAccel(double joystickY) {
    double newSpeed = Math.pow(joystickY, 3) * 1.6 + (0.17 * joystickY);
    return newSpeed;
  }
}
