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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.util.Limiter;

public class DriveTrainSubsystem extends SubsystemBase {

  private DriveTrainSubsystem m_drivetrainSubsystem;

  public DriveTrainSubsystem(DriveTrainSubsystem p_drivetrainSubsystem) {
    m_drivetrainSubsystem = p_drivetrainSubsystem;
  }

  // ODOMETRY
  private DifferentialDriveOdometry m_diffDriveOdometry;
  private AHRS m_navX = new AHRS(SPI.Port.kMXP);

  // MOTORS
  private CANSparkMax m_motorLeft1 =
      new CANSparkMax(MotorConstants.MOTOR_LEFT1, MotorType.kBrushless);
  private CANSparkMax m_motorLeft2 =
      new CANSparkMax(MotorConstants.MOTOR_LEFT2, MotorType.kBrushless);
  private CANSparkMax m_motorRight1 =
      new CANSparkMax(MotorConstants.MOTOR_RIGHT1, MotorType.kBrushless);
  private CANSparkMax m_motorRight2 =
      new CANSparkMax(MotorConstants.MOTOR_RIGHT2, MotorType.kBrushless);

  private RelativeEncoder m_encoderLeft1;
  private RelativeEncoder m_encoderLeft2;
  private RelativeEncoder m_encoderRight1;
  private RelativeEncoder m_encoderRight2;
  private Joystick m_leftJoystick;
  private Joystick m_rightJoystick;
  private JoystickButton m_driveBackwards;
  private MotorControllerGroup m_motorGroupRight =
      new MotorControllerGroup(m_motorRight1, m_motorRight2);
  private MotorControllerGroup m_motorGroupLeft =
      new MotorControllerGroup(m_motorLeft1, m_motorLeft2);

  /** Creates a new Drivetrain. */
  public DriveTrainSubsystem(Joystick p_leftJoystick, Joystick p_rightJoystick) {
    m_leftJoystick = p_leftJoystick;
    m_rightJoystick = p_rightJoystick;
    m_driveBackwards = new JoystickButton(m_rightJoystick, 13);

    m_encoderLeft1 = initializeMotor(m_motorLeft1, true);
    m_encoderLeft2 = initializeMotor(m_motorLeft2, true);
    m_encoderRight1 = initializeMotor(m_motorRight1, false);
    m_encoderRight2 = initializeMotor(m_motorRight2, false);

    m_diffDriveOdometry = new DifferentialDriveOdometry(getYawAsRotation(), getL1Pos(), getR1Pos());
    resetOdometry();
  }

  private RelativeEncoder initializeMotor(CANSparkMax p_motor, boolean p_inversion) {
    RelativeEncoder p_encoder;

    p_motor.restoreFactoryDefaults();
    p_motor.setIdleMode(IdleMode.kBrake);
    p_motor.setInverted(p_inversion);

    p_encoder = p_motor.getEncoder();
    p_encoder.setPositionConversionFactor(DriverConstants.ENCODER_POSITION_CONVERSION);
    p_encoder.setVelocityConversionFactor(DriverConstants.ENCODER_VELOCITY_CONVERSION);
    return p_encoder;
  }

  public void resetEncoders() {
    m_encoderRight1.setPosition(0);
    m_encoderRight2.setPosition(0);
    m_encoderLeft1.setPosition(0);
    m_encoderLeft2.setPosition(0);
  }

  public void resetOdometry() {
    resetEncoders();
    m_diffDriveOdometry.resetPosition(
        Rotation2d.fromDegrees(getYaw()),
        m_encoderLeft1.getPosition(),
        m_encoderRight1.getPosition(),
        m_diffDriveOdometry.getPoseMeters());
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
    return m_navX.getYaw() * -1;
  }

  public double getPitch() {
    return m_navX.getPitch();
  }

  public double getRoll() {
    return m_navX.getRoll();
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

  public void setVoltage(double voltage) {
    setVoltage(voltage, voltage);
  }

  public void setMotors(double leftMotorInput, double rightMotorInput) {
    m_motorGroupLeft.set(deadzone(leftMotorInput));
    m_motorGroupRight.set(deadzone(rightMotorInput));
  }

  public void smartSetMotors(double leftMotorInput, double rightMotorInput) {
    /* if both triggers are held, enable turbo mode */
    if (m_leftJoystick.getTrigger() && m_rightJoystick.getTrigger()) {
      m_motorGroupLeft.set(turboAccel(deadzone(leftMotorInput)));
      m_motorGroupRight.set(turboAccel(deadzone(rightMotorInput)));
    }
    /* if the top of the joystick is held  */
    else if (m_driveBackwards.getAsBoolean()) {
      m_motorGroupLeft.set(-deadzone(rightMotorInput));
      m_motorGroupRight.set(-deadzone(leftMotorInput));
    } else {
      m_motorGroupLeft.set(deadzone(leftMotorInput));
      m_motorGroupRight.set(deadzone(rightMotorInput));
    }
  }

  public double deadzone(double joystickY) {
    return Limiter.deadzone(joystickY, DrivetrainConstants.JOYSTICK_DEADZONE);
  }

  public static double turboAccel(double joystickY) {
    return (2 * DriverConstants.MAXSPEED * joystickY) / (1 + Math.abs(joystickY));
  }

  public static double slowAccel(double joystickY) {
    return Math.pow(joystickY, 3) * 1.6 + (0.17 * joystickY);
  }

  public AHRS getNavX() {
    return m_navX;
  }

  @Override
  public void periodic() {
    m_diffDriveOdometry.update(
        getYawAsRotation(), m_encoderLeft1.getPosition(), m_encoderRight1.getPosition());
  }

  public void turnMotor(double limelightAdjust) {
  }

public void theta(double d) {
}
}
