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
import edu.wpi.first.wpilibj.Joystick;
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
  private Joystick m_leftJoystick;
  private Joystick m_rightJoystick;
  MotorControllerGroup m_motorGroupRight = new MotorControllerGroup(m_motorRight1, m_motorRight2);
  MotorControllerGroup m_motorGroupLeft = new MotorControllerGroup(m_motorLeft1, m_motorLeft2);

  /** Creates a new Drivetrain. */
  public DriveTrainSubsystem(Joystick p_leftJoystick, Joystick p_rightJoystick) {

    m_leftJoystick = p_leftJoystick;
    m_rightJoystick = p_rightJoystick;

    m_encoderLeft1 = initializeMotor(m_motorLeft1, true);
    m_encoderLeft2 = initializeMotor(m_motorLeft2, true);
    m_encoderRight1 = initializeMotor(m_motorRight1, false);
    m_encoderRight2 = initializeMotor(m_motorRight2, false);

    m_diffDriveOdometry = new DifferentialDriveOdometry(getYawAsRotation(), getL1Pos(), getR1Pos());
    resetOdometry(m_diffDriveOdometry.getPoseMeters());
    setMotors(0.3, 0);
  }

  private RelativeEncoder initializeMotor(CANSparkMax p_motor, boolean p_inversion) {
    RelativeEncoder p_encoder;

    p_motor.restoreFactoryDefaults();
    p_motor.setIdleMode(IdleMode.kBrake);
    p_motor.setInverted(p_inversion);
    p_motor.burnFlash();

    p_encoder = p_motor.getEncoder();
    p_encoder.setPositionConversionFactor(Constants.DriverConstants.ENCODER_POSITION_CONVERSION);
    p_encoder.setVelocityConversionFactor(Constants.DriverConstants.ENCODER_VELOCITY_CONVERSION);
    return p_encoder;
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
    boolean turbo = m_leftJoystick.getTrigger() && m_rightJoystick.getTrigger();

    double leftMotorSpeed = linearAccel(deadzone(leftMotorInput));
    double rightMotorSpeed = linearAccel(deadzone(rightMotorInput));

    if (turbo) {
      leftMotorSpeed = turboAccel(deadzone(leftMotorInput));
      rightMotorSpeed = turboAccel(deadzone(rightMotorInput));
    }
    m_motorGroupLeft.set(leftMotorSpeed);
    m_motorGroupRight.set(rightMotorSpeed);
  }

  public double deadzone(double joystickY) {
    double absJoystickY = Math.abs(joystickY);
    if (absJoystickY <= Constants.VariableConstants.DEADZONE) {
      return 0.0;
    } else {
      return joystickY;
    }
  }

  public static double linearAccel(double joystickY) {
    return joystickY;
  }

  public static double turboAccel(double joystickY) {
    return (2 * Constants.DriverConstants.MAXSPEED * joystickY) / (1 + Math.abs(joystickY));
  }

  public static double slowAccel(double joystickY) {
    return Math.pow(joystickY, 3) * 1.6 + (0.17 * joystickY);
  }

  @Override
  public void periodic() {
    m_diffDriveOdometry.update(
        getYawAsRotation(), m_encoderLeft1.getPosition(), m_encoderRight1.getPosition());
  }
}
