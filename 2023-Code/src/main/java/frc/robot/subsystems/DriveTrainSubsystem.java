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
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.MotorConstants;

public class DriveTrainSubsystem extends SubsystemBase {

  double cycle = 0;
  double straightMotorSpeed = -0.2;
  double turnMotorSpeed = 0.05;
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
  private MotorControllerGroup m_motorGroupRight =
      new MotorControllerGroup(m_motorRight1, m_motorRight2);
  private MotorControllerGroup m_motorGroupLeft =
      new MotorControllerGroup(m_motorLeft1, m_motorLeft2);

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

    m_encoderRight1.setPosition(0);
    m_encoderRight2.setPosition(0);
    m_encoderLeft1.setPosition(0);
    m_encoderLeft2.setPosition(0);
    setMotors(0.3, 0);
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

  public void setMotorsStraight(double m_bothMotorSpeed, double placeholder) {
    m_motorGroupRight.set(m_bothMotorSpeed * -1);
    m_motorGroupLeft.set(m_bothMotorSpeed * 1);
  }

  public double deadzone(double joystickY) {
    double absJoystickY = Math.abs(joystickY);
    if (absJoystickY <= DrivetrainConstants.JOYSTICK_DEADZONE) {
      return 0.0;
    } else {
      return joystickY;
    }
  }

  public void printPosition() {
    if (cycle % 20 == 0) {
      System.out.println("Positions are " + getL1Pos() + " " + getR1Pos());
    }
  }

  public boolean setMotorPosition(
      double motorset, double p_rmotorPosition, double p_lmotorPosition) {
    motorset = motorset * 15;
    double rmotorPosition = getR1Pos();
    double lmotorPosition = getL1Pos();
    boolean stop = false;

    if (rmotorPosition < p_rmotorPosition + motorset
        && lmotorPosition < p_lmotorPosition + motorset) {
      m_motorGroupRight.set(straightMotorSpeed);
      m_motorGroupLeft.set(straightMotorSpeed);
      // System.out.println("Setpoint " + motorset);
      // System.out.println("starting position " + p_rmotorPosition);
      // System.out.println("current position " + rmotorPosition);
    } else {
      m_motorGroupRight.set(0);
      m_motorGroupLeft.set(0);
      stop = true;
    }
    return stop;
    /*
    while (getR1Pos() < rmotorPosition + rmotorset && getL1Pos() < lmotorPosition + lmotorset) {
      m_motorGroupRight.set(0.3);
      m_motorGroupLeft.set(0.3);
    }
    */
  }

  public boolean turnMotor(double motorset, double rmotorPosition, double lmotorPosition) {
    boolean stop = false;
    if (motorset > 0) {
      stop = turnMotorRight(Math.abs(motorset), rmotorPosition, lmotorPosition);
      // System.out.println("turning motor right");
    } else if (motorset < 0) {
      stop = turnMotorLeft(Math.abs(motorset), rmotorPosition, lmotorPosition);
      // System.out.println("turning motor left");
    }
    return stop;
  }

  public boolean turnMotorRight(double motorset, double p_rmotorPosition, double p_lmotorPosition) {
    motorset = motorset * 20 / Math.PI;
    double rmotorPosition = getR1Pos();
    double lmotorPosition = getL1Pos();
    boolean stop = false;

    if (rmotorPosition > p_rmotorPosition - motorset
        && lmotorPosition < p_lmotorPosition + motorset) {
      // System.out.println("setpoint " + (p_rmotorPosition - motorset));
      // System.out.println("initial position " + p_rmotorPosition);
      // System.out.println("current position r" + rmotorPosition);
      m_motorGroupRight.set(-1 * turnMotorSpeed);
      m_motorGroupLeft.set(turnMotorSpeed);
    } else {
      m_motorGroupRight.set(0);

      m_motorGroupLeft.set(0);
      stop = true;
      // System.out.println("rmotor position " + rmotorPosition);
      // System.out.println("lmotor position " + lmotorPosition);
      // System.out.println("setpoint " + (p_rmotorPosition - motorset));

    }
    return stop;

    /*
    while (getR1Pos() > rmotorPosition - motorset && getL1Pos() < lmotorPosition + motorset) {
      m_motorGroupRight.set(-0.05);
      m_motorGroupLeft.set(0.05);
    }
    */
  }

  public boolean turnMotorLeft(double motorset, double p_rmotorPosition, double p_lmotorPosition) {
    motorset = motorset * 20 / Math.PI;
    double rmotorPosition = getR1Pos();
    double lmotorPosition = getL1Pos();
    boolean stop = false;

    if (rmotorPosition < p_rmotorPosition + motorset
        && lmotorPosition > p_lmotorPosition - motorset) {
      // System.out.println("setpoint " + (p_rmotorPosition - motorset));
      // System.out.println("initial position " + p_rmotorPosition);
      // System.out.println("current position r " + rmotorPosition);
      m_motorGroupRight.set(turnMotorSpeed);
      m_motorGroupLeft.set(-1 * turnMotorSpeed);
    } else {
      m_motorGroupRight.set(0);
      m_motorGroupLeft.set(0);
      stop = true;
      // System.out.println("rmotor position " + rmotorPosition);
      // System.out.println("lmotor position " + lmotorPosition);
      // System.out.println("setpoint " + (p_rmotorPosition + motorset));
    }
    return stop;

    /*
    while (getR1Pos() < rmotorPosition + motorset && getL1Pos() > lmotorPosition - motorset) {
      m_motorGroupRight.set(0.05);
      m_motorGroupLeft.set(-0.05);
    }
    */
  }

  public static double linearAccel(double joystickY) {
    return joystickY;
  }

  public static double turboAccel(double joystickY) {
    return (2 * DriverConstants.MAXSPEED * joystickY) / (1 + Math.abs(joystickY));
  }

  public static double slowAccel(double joystickY) {
    return Math.pow(joystickY, 3) * 1.6 + (0.17 * joystickY);
  }

  @Override
  public void periodic() {
    m_diffDriveOdometry.update(
        getYawAsRotation(), m_encoderLeft1.getPosition(), m_encoderRight1.getPosition());
    // cycle++;
    // printPosition();
    // System.out.println(getR1Pos() / 0.04987278);
    // System.out.println(m_encoderRight1.getPositionConversionFactor());
  }
}
