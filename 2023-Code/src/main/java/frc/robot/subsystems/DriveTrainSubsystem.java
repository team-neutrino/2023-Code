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
  private CANSparkMax m_motorRight1 =
      new CANSparkMax(Constants.MotorConstants.RMOTOR1, MotorType.kBrushless);
  private CANSparkMax m_motorRight2 =
      new CANSparkMax(Constants.MotorConstants.RMOTOR2, MotorType.kBrushless);
  private CANSparkMax m_motorLeft1 =
      new CANSparkMax(Constants.MotorConstants.LMOTOR1, MotorType.kBrushless);
  private CANSparkMax m_motorLeft2 =
      new CANSparkMax(Constants.MotorConstants.LMOTOR2, MotorType.kBrushless);

  private RelativeEncoder m_encoderRight1;
  private RelativeEncoder m_encoderRight2;
  private RelativeEncoder m_encoderLeft1;
  private RelativeEncoder m_encoderLeft2;

  MotorControllerGroup m_rmotors = new MotorControllerGroup(m_motorRight1, m_motorRight2);
  MotorControllerGroup m_lmotors = new MotorControllerGroup(m_motorLeft1, m_motorLeft2);

  /** Creates a new Drivetrain. */
  public DriveTrainSubsystem() {
    m_motorRight1.restoreFactoryDefaults();
    m_motorRight2.restoreFactoryDefaults();
    m_motorLeft1.restoreFactoryDefaults();
    m_motorRight2.restoreFactoryDefaults();

    m_motorRight1.setIdleMode(IdleMode.kBrake);
    m_motorRight2.setIdleMode(IdleMode.kBrake);
    m_motorLeft1.setIdleMode(IdleMode.kBrake);
    m_motorLeft2.setIdleMode(IdleMode.kBrake);

    m_motorRight1.setInverted(false);
    m_motorRight2.setInverted(false);
    m_motorLeft1.setInverted(true);
    m_motorLeft2.setInverted(true);

    m_motorRight1.burnFlash();
    m_motorRight2.burnFlash();
    m_motorLeft1.burnFlash();
    m_motorLeft2.burnFlash();

    m_encoderRight1 = m_motorRight1.getEncoder();
    m_encoderRight2 = m_motorRight2.getEncoder();
    m_encoderLeft1 = m_motorLeft1.getEncoder();
    m_encoderLeft2 = m_motorLeft2.getEncoder();

    m_encoderRight1.setPositionConversionFactor(Constants.DriverConstants.ENCODER_CONVERSION);
    m_encoderRight2.setPositionConversionFactor(Constants.DriverConstants.ENCODER_CONVERSION);
    m_encoderLeft1.setPositionConversionFactor(Constants.DriverConstants.ENCODER_CONVERSION);
    m_encoderLeft2.setPositionConversionFactor(Constants.DriverConstants.ENCODER_CONVERSION);

    m_encoderRight1.setVelocityConversionFactor(Constants.DriverConstants.ENCODER_CONVERSION / 60);
    m_encoderRight2.setVelocityConversionFactor(Constants.DriverConstants.ENCODER_CONVERSION / 60);
    m_encoderLeft1.setVelocityConversionFactor(Constants.DriverConstants.ENCODER_CONVERSION / 60);
    m_encoderLeft2.setVelocityConversionFactor(Constants.DriverConstants.ENCODER_CONVERSION / 60);

    m_diffDriveOdometry =
        new DifferentialDriveOdometry(getGyroYawAsRotation(), getL1Pos(), getR1Pos());
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
        Rotation2d.fromDegrees(getGyroYaw()),
        m_encoderLeft1.getPosition(),
        m_encoderRight1.getPosition(),
        pose);
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
        getGyroYawAsRotation(), m_encoderLeft1.getPosition(), m_encoderRight1.getPosition());
  }
}
