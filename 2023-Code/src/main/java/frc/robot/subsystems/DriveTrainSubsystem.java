// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {

<<<<<<< HEAD
  double cycle = 0;
=======
  // ODOMETRY
  private DifferentialDriveOdometry m_diffDriveOdometry;
>>>>>>> main
  private AHRS m_navX = new AHRS(SPI.Port.kMXP);

  // MOTORS
  private CANSparkMax m_motorRight1 =
      new CANSparkMax(Constants.MotorConstants.MOTOR_RIGHT1, MotorType.kBrushless);
  private CANSparkMax m_motorRight2 =
      new CANSparkMax(Constants.MotorConstants.MOTOR_RIGHT2, MotorType.kBrushless);
  private CANSparkMax m_motorLeft1 =
      new CANSparkMax(Constants.MotorConstants.MOTOR_LEFT1, MotorType.kBrushless);
  private CANSparkMax m_motorLeft2 =
      new CANSparkMax(Constants.MotorConstants.MOTOR_LEFT2, MotorType.kBrushless);

  private RelativeEncoder m_encoderRight1;
  private RelativeEncoder m_encoderRight2;
  private RelativeEncoder m_encoderLeft1;
  private RelativeEncoder m_encoderLeft2;

  MotorControllerGroup m_motorGroupRight = new MotorControllerGroup(m_motorRight1, m_motorRight2);
  MotorControllerGroup m_motorGroupLeft = new MotorControllerGroup(m_motorLeft1, m_motorLeft2);

  /** Creates a new Drivetrain. */
  public DriveTrainSubsystem() {
    m_motorLeft1.restoreFactoryDefaults();
    m_motorLeft2.restoreFactoryDefaults();
    m_motorRight1.restoreFactoryDefaults();
    m_motorRight2.restoreFactoryDefaults();

    m_motorLeft1.setIdleMode(IdleMode.kBrake);
    m_motorLeft2.setIdleMode(IdleMode.kBrake);
    m_motorRight1.setIdleMode(IdleMode.kBrake);
    m_motorRight2.setIdleMode(IdleMode.kBrake);

    m_motorLeft1.setInverted(true);
    m_motorLeft2.setInverted(true);
    m_motorRight1.setInverted(false);
    m_motorRight2.setInverted(false);

<<<<<<< HEAD
    m_encoderR1 = m_rmotor1.getEncoder();
    m_encoderR2 = m_rmotor2.getEncoder();
    m_encoderL1 = m_lmotor1.getEncoder();
    m_encoderL2 = m_lmotor2.getEncoder();

    m_encoderR1.setPosition(0);
    m_encoderR2.setPosition(0);
    m_encoderL1.setPosition(0);
    m_encoderL2.setPosition(0);
=======
    m_motorLeft1.burnFlash();
    m_motorLeft2.burnFlash();
    m_motorRight1.burnFlash();
    m_motorRight2.burnFlash();

    m_encoderLeft1 = m_motorLeft1.getEncoder();
    m_encoderLeft2 = m_motorLeft2.getEncoder();
    m_encoderRight1 = m_motorRight1.getEncoder();
    m_encoderRight2 = m_motorRight2.getEncoder();

    m_encoderLeft1.setPositionConversionFactor(
        Constants.DriverConstants.ENCODER_POSITION_CONVERSION);
    m_encoderLeft2.setPositionConversionFactor(
        Constants.DriverConstants.ENCODER_POSITION_CONVERSION);
    m_encoderRight1.setPositionConversionFactor(
        Constants.DriverConstants.ENCODER_POSITION_CONVERSION);
    m_encoderRight2.setPositionConversionFactor(
        Constants.DriverConstants.ENCODER_POSITION_CONVERSION);

    m_encoderLeft1.setVelocityConversionFactor(
        Constants.DriverConstants.ENCODER_VELOCITY_CONVERSION);
    m_encoderLeft2.setVelocityConversionFactor(
        Constants.DriverConstants.ENCODER_VELOCITY_CONVERSION);
    m_encoderRight1.setVelocityConversionFactor(
        Constants.DriverConstants.ENCODER_VELOCITY_CONVERSION);
    m_encoderRight2.setVelocityConversionFactor(
        Constants.DriverConstants.ENCODER_VELOCITY_CONVERSION);

    m_diffDriveOdometry = new DifferentialDriveOdometry(getYawAsRotation(), getL1Pos(), getR1Pos());
    resetOdometry(m_diffDriveOdometry.getPoseMeters());
  }

  public void resetEncoders() {
    m_encoderRight1.setPosition(0);
    m_encoderRight2.setPosition(0);
    m_encoderLeft1.setPosition(0);
    m_encoderLeft2.setPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_navX.reset();
    m_diffDriveOdometry.resetPosition(
        Rotation2d.fromDegrees(getYaw()),
        m_encoderLeft1.getPosition(),
        m_encoderRight1.getPosition(),
        pose);
>>>>>>> main
  }

  public double getR1Pos() {
    return m_encoderRight1.getPosition();
  }

  public double getR2Pos() {
    return m_encoderRight2.getPosition();
  }

  public double getL1Pos() {
    return m_encoderLeft1.getPosition();
  }

  public double getL2Pos() {
    return m_encoderLeft2.getPosition();
  }

  public double getR1Vel() {
    return m_encoderRight1.getVelocity();
  }

  public double getR2Vel() {
    return m_encoderRight2.getVelocity();
  }

  public double getL1Vel() {
    return m_encoderLeft1.getVelocity();
  }

  public double getL2Vel() {
    return m_encoderLeft2.getVelocity();
  }

  public double getYaw() {
    return m_navX.getYaw();
  }

  public double getPitch() {
    return m_navX.getPitch();
  }

  private Rotation2d getYawAsRotation() {
    return Rotation2d.fromDegrees(getYaw());
  }

  public Pose2d getPose2d() {
    return m_diffDriveOdometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getDriveWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getL1Vel(), getR1Vel());
  }

  public void setVoltage(double leftVoltage, double rightVoltage) {
    m_motorGroupLeft.setVoltage(leftVoltage);
    m_motorGroupRight.setVoltage(rightVoltage);
  }

  public void setMotors(double leftMotorInput, double rightMotorInput) {
    double leftMotorSpeed = linearAccel(deadzone(leftMotorInput));
    double rightMotorSpeed = linearAccel(deadzone(rightMotorInput));
    m_motorGroupLeft.set(leftMotorSpeed);
    m_motorGroupRight.set(rightMotorSpeed);
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

<<<<<<< HEAD
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

=======
>>>>>>> main
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

  @Override
  public void periodic() {
    m_diffDriveOdometry.update(
        getYawAsRotation(), m_encoderLeft1.getPosition(), m_encoderRight1.getPosition());
  }
}
