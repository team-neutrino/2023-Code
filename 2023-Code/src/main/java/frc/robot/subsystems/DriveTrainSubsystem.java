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

  // ODOMETRY
  private DifferentialDriveOdometry m_diffDriveOdometry;
  private AHRS m_navX = new AHRS(SPI.Port.kMXP);

  // MOTORS
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

    m_rmotor1.setIdleMode(IdleMode.kBrake);
    m_rmotor2.setIdleMode(IdleMode.kBrake);
    m_lmotor1.setIdleMode(IdleMode.kBrake);
    m_lmotor2.setIdleMode(IdleMode.kBrake);

    m_rmotor1.setInverted(false);
    m_rmotor2.setInverted(false);
    m_lmotor1.setInverted(true);
    m_lmotor2.setInverted(true);

    m_rmotor1.burnFlash();
    m_rmotor2.burnFlash();
    m_lmotor1.burnFlash();
    m_lmotor2.burnFlash();

    m_encoderR1 = m_rmotor1.getEncoder();
    m_encoderR2 = m_rmotor2.getEncoder();
    m_encoderL1 = m_lmotor1.getEncoder();
    m_encoderL2 = m_lmotor2.getEncoder();

    m_encoderR1.setPositionConversionFactor(Constants.DriverContants.ENCODER_CONVERSION);
    m_encoderR1.setPositionConversionFactor(Constants.DriverContants.ENCODER_CONVERSION);
    m_encoderR1.setPositionConversionFactor(Constants.DriverContants.ENCODER_CONVERSION);
    m_encoderR1.setPositionConversionFactor(Constants.DriverContants.ENCODER_CONVERSION);

    m_encoderR1.setVelocityConversionFactor(Constants.DriverContants.ENCODER_CONVERSION / 60);
    m_encoderR1.setVelocityConversionFactor(Constants.DriverContants.ENCODER_CONVERSION / 60);
    m_encoderR1.setVelocityConversionFactor(Constants.DriverContants.ENCODER_CONVERSION / 60);
    m_encoderR1.setVelocityConversionFactor(Constants.DriverContants.ENCODER_CONVERSION / 60);

    m_diffDriveOdometry =
        new DifferentialDriveOdometry(getGyroYawAsRotation(), getL1Pos(), getR1Pos());
    resetOdometry(m_diffDriveOdometry.getPoseMeters());
  }

  public void resetEncoders() {
    m_encoderR1.setPosition(0);
    m_encoderR2.setPosition(0);
    m_encoderL1.setPosition(0);
    m_encoderL2.setPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_navX.reset();
    m_diffDriveOdometry.resetPosition(
        Rotation2d.fromDegrees(getGyroYaw()),
        m_encoderL1.getPosition(),
        m_encoderR1.getPosition(),
        pose);
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

  public double getGyroYaw() {
    return -1 * m_navX.getYaw();
  }

  public double getPitch() {
    return m_navX.getPitch();
  }

  private Rotation2d getGyroYawAsRotation() {
    return Rotation2d.fromDegrees(getGyroYaw());
  }

  public Pose2d getPose2d() {
    return m_diffDriveOdometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getDriveWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getL1Vel(), getR1Vel());
  }

  public void setVoltage(double leftVoltage, double rightVoltage) {
    m_rmotors.setVoltage(rightVoltage);
    m_lmotors.setVoltage(leftVoltage);
  }

  public void setMotors(double rightMotorInput, double leftMotorInput) {
    double leftMotorSpeed = linearAccel(deadzone(leftMotorInput));
    double rightMotorSpeed = linearAccel(deadzone(rightMotorInput));
    m_rmotors.set(rightMotorSpeed);
    m_lmotors.set(leftMotorSpeed);
  }

  public double deadzone(double joystickY) {
    joystickY = Math.abs(joystickY);
    if (joystickY <= Constants.VariableConstants.DEADZONE) {
      return 0.0;
    } else if (joystickY >= 1.0) {
      return 1.0;
    } else {
      return joystickY;
    }
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

  @Override
  public void periodic() {
    System.out.println(getPose2d());
    m_diffDriveOdometry.update(
        getGyroYawAsRotation(), m_encoderL1.getPosition(), m_encoderR1.getPosition());
  }
}
