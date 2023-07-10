// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.util.Limiter;
import frc.robot.util.PoseTriplet;
import java.util.ArrayList;
import java.util.Arrays;

public class DriveTrainSubsystem extends SubsystemBase {

  private DriveTrainSubsystem m_drivetrainSubsystem;
  private boolean autonRunning;

  public DriveTrainSubsystem(DriveTrainSubsystem p_drivetrainSubsystem) {
    m_drivetrainSubsystem = p_drivetrainSubsystem;
  }

  // SIMULATION
  double encoderSimLeft = 0;
  double encoderSimRight = 0;
  double navXSim = 0;
  double cycle = 0;
  int simDevice = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
  SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(simDevice, "Yaw"));
  Field2d m_field = new Field2d();
  Pose2d dumbPose = new Pose2d(3, 1, Rotation2d.fromDegrees(0));

  private ArrayList<PoseTriplet> toGamePieceArray =
      new ArrayList<PoseTriplet>(
          Arrays.asList(
              new PoseTriplet(0, 0, 0),
              new PoseTriplet(2.7, -0.10, -15.08),
              new PoseTriplet(4.08, -0.22, -3.12)));

  ArrayList<PoseTriplet> runThatBack =
      new ArrayList<PoseTriplet>(
          Arrays.asList(
              new PoseTriplet(4.08, -0.22, -3.12),
              new PoseTriplet(1.3, -0.08, 0.16),
              new PoseTriplet(-.3, -.31, -.32)));

  ArrayList<PoseTriplet> test1 =
      new ArrayList<PoseTriplet>(Arrays.asList(new PoseTriplet(0, 0, 0), new PoseTriplet(4, 0, 0)));

  // ODOMETRY
  private DifferentialDriveOdometry m_diffDriveOdometry;
  private AHRS m_navX = new AHRS(SPI.Port.kMXP);

  private DifferentialDrivetrainSim m_diffDriveSim;
  DifferentialDrivetrainSim m_diffDriveSimTest;
  DifferentialDriveOdometry m_diffDriveOdometrySim;

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

  private XboxController m_driverController;

  /** Creates a new Drivetrain. */
  public DriveTrainSubsystem(
      Joystick p_leftJoystick, Joystick p_rightJoystick, XboxController p_driverController) {
    m_leftJoystick = p_leftJoystick;
    m_rightJoystick = p_rightJoystick;

    m_driverController = p_driverController;

    m_driveBackwards = new JoystickButton(m_rightJoystick, 13);

    m_encoderLeft1 = initializeMotor(m_motorLeft1, true);
    m_encoderLeft2 = initializeMotor(m_motorLeft2, true);
    m_encoderRight1 = initializeMotor(m_motorRight1, false);
    m_encoderRight2 = initializeMotor(m_motorRight2, false);

    m_diffDriveOdometry = new DifferentialDriveOdometry(getYawAsRotation(), getL1Pos(), getR1Pos());
    resetOdometry();

    //initial pose??? (passed into the constructor for the odometrysim object as the last parameter... I think this "overrides" everything
    //else?)
    Pose2d newPose = new Pose2d(3, 3, Rotation2d.fromDegrees(0));

    m_diffDriveSim =
        new DifferentialDrivetrainSim(DCMotor.getNEO(2), 8, 6, 50, 0.0635, 0.635, null);

    m_diffDriveSimTest =
        DifferentialDrivetrainSim.createKitbotSim(
            KitbotMotor.kDualCIMPerSide, KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);

    m_diffDriveOdometrySim =
        new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0, 0, newPose);

    SmartDashboard.putData("Field", m_field);
    // m_field.getObject("trajectory").setTrajectory(AutonomousUtil.generateTrajectoryFromPoses(toGamePieceArray,
    // TrajectoryConfigConstants.K_LESS_SPEED_FORWARD_CONFIG));

    // m_field.getObject("trajectory").setTrajectory(AutonomousUtil.generateTrajectoryFromPoses(runThatBack,
    // TrajectoryConfigConstants.K_LESS_SPEED_FORWARD_CONFIG));

    //m_field.setRobotPose(newPose);

    //this seems to do the trick :)
    m_field.getRobotObject().close();
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

  public CANSparkMax getMotorRight1() {
    return m_motorRight1;
  }

  public CANSparkMax getMotorRight2() {
    return m_motorRight2;
  }

  public CANSparkMax getMotorLeft1() {
    return m_motorLeft1;
  }

  public CANSparkMax getMotorLeft2() {
    return m_motorLeft2;
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

  public void setSimInputs(double leftInputs, double rightInputs) {
    m_diffDriveSim.setInputs(leftInputs, rightInputs);
  }

  @Override
  public void periodic() {
    m_diffDriveOdometry.update(
        getYawAsRotation(), m_encoderLeft1.getPosition(), m_encoderRight1.getPosition());

    m_diffDriveOdometrySim.update(Rotation2d.fromDegrees(navXSim), encoderSimLeft, encoderSimRight);

    m_field.getObject("Sim Robot").setPose(m_diffDriveOdometrySim.getPoseMeters());
    //m_field.setRobotPose(m_diffDriveOdometrySim.getPoseMeters());
  }

  public void setAutonRunning() {
    autonRunning = true;
  }

  //find a way to implement this (currently not in use)
  public void setAutonOff() {
    autonRunning = false;
  }

  @Override
  public void simulationPeriodic() {

    // can also use m_motorGroupLeft.get() to use through joysticks, etc.
    if (!autonRunning) {
      m_diffDriveSim.setInputs(
          Limiter.deadzone(m_driverController.getLeftY(), 0.1)
              * RobotController.getInputVoltage()
              * -1,
          Limiter.deadzone(m_driverController.getRightY(), 0.1)
              * RobotController.getInputVoltage()
              * -1);
    }

    // if (!autonRunning) {
    //   m_diffDriveSim.setInputs(
    //       Limiter.deadzone(m_motorGroupLeft.get() * RobotController.getInputVoltage(), 0.1),
    //       Limiter.deadzone(m_motorGroupRight.get() * RobotController.getInputVoltage(), 0.1));
    // }

    cycle++;
    simulationPrint();

    m_diffDriveSim.update(0.02);

    encoderSimLeft = m_diffDriveSim.getLeftPositionMeters();
    encoderSimRight = m_diffDriveSim.getRightPositionMeters();
    angle.set(m_diffDriveSim.getHeading().getDegrees());
    navXSim = m_diffDriveSim.getHeading().getDegrees();

    /*
    m_diffDriveSimTest.setInputs(
        Limiter.deadzone(m_driverController.getLeftY(), 0.1) * RobotController.getInputVoltage() * -1,
        Limiter.deadzone(m_driverController.getRightY(), 0.1) * RobotController.getInputVoltage() * -1);

    m_diffDriveSimTest.update(0.02);

    encoderSimLeft = m_diffDriveSimTest.getLeftPositionMeters();
    encoderSimRight = m_diffDriveSimTest.getRightPositionMeters();
    navXSim = m_diffDriveSimTest.getHeading().getDegrees();
    /* */
  }

  public void simulationPrint() {
    if (cycle % 40 == 0) {
      // System.out.println("left stick: " + m_driverController.getLeftY());
      // System.out.println("right stick: " + m_driverController.getRightY());

      // System.out.println("motor output left: " + Limiter.deadzone(m_driverController.getLeftY(),
      // 0.1) * RobotController.getInputVoltage());
      // System.out.println("motor output right: " +
      // Limiter.deadzone(m_driverController.getRightY(), 0.1) * RobotController.getInputVoltage());

      // System.out.println("left velocity: " + m_diffDriveSim.getLeftVelocityMetersPerSecond());
      // System.out.println("right velocity: " + m_diffDriveSim.getRightVelocityMetersPerSecond());

      // System.out.println("rio voltage " + RobotController.getInputVoltage());
      // System.out.println("yaw: " + getYaw());
      // System.out.println("sim output angle: " + m_diffDriveSim.getHeading().getDegrees());

      // System.out.println("left encoder: " + encoderSimLeft);
      // System.out.println("right encoder: " + encoderSimRight);
    }
  }
}
