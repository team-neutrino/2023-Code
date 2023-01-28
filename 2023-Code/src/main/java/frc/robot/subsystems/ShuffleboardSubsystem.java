// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.util.DriverStationInfo;

public class ShuffleboardSubsystem extends SubsystemBase {
  private ShuffleboardTab m_drivestationTab;
  private ShuffleboardTab m_debugTab;
  private GenericEntry m_driveTrainVariables[] = new GenericEntry[15];
  private GenericEntry m_limelightVariables[] = new GenericEntry[9];
  private GenericEntry m_driverStationInfoVariables[] = new GenericEntry[9];
  private GenericEntry m_scoringVariables[] = new GenericEntry[11];
  private GenericEntry m_armVariables[] = new GenericEntry[4];
  private GenericEntry m_intakeVariables[] = new GenericEntry[4];

  private SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  private DriveTrainSubsystem m_driveTrain;
  private LimelightSubsystem m_limelight;
  private ScoringSubsystem m_scoring;
  // private EndGameSubsystem m_endGame;
  private DriverStationInfo m_driverStationInfo;
  private ArmSubsystem m_arm;
  private HttpCamera LLFeed;
  private IntakeSubsystem m_intake;

  public ShuffleboardSubsystem(
      DriverStationInfo p_driverStationInfo,
      DriveTrainSubsystem p_driveTrain,
      ScoringSubsystem p_scoring,
      LimelightSubsystem p_limelight,
      ArmSubsystem p_arm,
      IntakeSubsystem p_intake) {

    m_driveTrain = p_driveTrain;
    m_limelight = p_limelight;
    m_scoring = p_scoring;
    m_arm = p_arm;
    m_intake = p_intake;
    m_driverStationInfo = p_driverStationInfo;
    m_drivestationTab = Shuffleboard.getTab("Driverstation Tab");
    m_debugTab = Shuffleboard.getTab("Debug Tab");

    setUpSelector();
    driveStationTab();
    debugTab();
  }

  public ShuffleboardSubsystem() {}

  @Override
  public void periodic() {
    m_driveTrainVariables[0].setDouble(m_driveTrain.getR1Pos());
    m_driveTrainVariables[1].setDouble(m_driveTrain.getR2Pos());
    m_driveTrainVariables[2].setDouble(m_driveTrain.getL1Pos());
    m_driveTrainVariables[3].setDouble(m_driveTrain.getL2Pos());
    m_driveTrainVariables[4].setDouble(m_driveTrain.getR1Vel());
    m_driveTrainVariables[5].setDouble(m_driveTrain.getR2Vel());
    m_driveTrainVariables[6].setDouble(m_driveTrain.getL1Vel());
    m_driveTrainVariables[7].setDouble(m_driveTrain.getL2Vel());

    m_scoringVariables[2].setBoolean(m_scoring.getSolenoidValue());

    m_driverStationInfoVariables[0].setDouble(m_driverStationInfo.getMatchTime());

    m_armVariables[0].setDouble(m_arm.getPosition());
    m_armVariables[1].setDouble(m_arm.getVoltage());

    m_intakeVariables[0].setDouble(m_intake.getWheelsEncoder());
    m_intakeVariables[1].setDouble(m_intake.getUpDownEncoder());

    m_limelightVariables[0].setDouble(m_limelight.getDistance());
    m_limelightVariables[1].setDouble(m_limelight.getID());
    m_limelightVariables[2].setDouble(m_limelight.getTx());
    m_limelightVariables[3].setDouble(m_limelight.getTy());
    m_limelightVariables[4].setBoolean(m_limelight.getTv());
  }

  private void driveStationTab() {
    m_drivestationTab = Shuffleboard.getTab("Drivestation Tab123");

    m_driveTrainVariables[0] =
        m_drivestationTab
            .add("Right Motor 1 Position", 0)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();

    m_driveTrainVariables[1] =
        m_drivestationTab
            .add("Right Motor 2 Position", 0)
            .withPosition(0, 1)
            .withSize(1, 1)
            .getEntry();

    m_driveTrainVariables[2] =
        m_drivestationTab
            .add("Left Motor 1 Position", 0)
            .withPosition(0, 2)
            .withSize(1, 1)
            .getEntry();

    m_driveTrainVariables[3] =
        m_drivestationTab
            .add("Left Motor 2 Position", 0)
            .withPosition(0, 3)
            .withSize(1, 1)
            .getEntry();

    m_driveTrainVariables[4] =
        m_drivestationTab
            .add("Right Motor 1 Velocity", 0)
            .withPosition(0, 3)
            .withSize(1, 1)
            .getEntry();

    m_driveTrainVariables[5] =
        m_drivestationTab
            .add("Right Motor 2 Velocity", 0)
            .withPosition(1, 3)
            .withSize(1, 1)
            .getEntry();

    m_driveTrainVariables[6] =
        m_drivestationTab
            .add("Left Motor 1 Velocity", 0)
            .withPosition(1, 2)
            .withSize(1, 1)
            .getEntry();

    m_driveTrainVariables[7] =
        m_drivestationTab
            .add("Left Motor 2 Velocity", 0)
            .withPosition(1, 1)
            .withSize(1, 1)
            .getEntry();

    m_scoringVariables[2] =
        m_drivestationTab.add("Grabber Release", 0).withPosition(2, 0).withSize(1, 1).getEntry();

    m_scoringVariables[4] =
        m_drivestationTab.add("Intake Down ", 0).withPosition(3, 0).withSize(1, 1).getEntry();

    m_scoringVariables[3] =
        m_drivestationTab.add("Squeeze", 0).withPosition(4, 0).withSize(1, 1).getEntry();

    m_limelightVariables[0] =
        m_drivestationTab.add("Distance", 0).withPosition(5, 0).withSize(1, 1).getEntry();

    m_driverStationInfoVariables[0] =
        m_drivestationTab.add("Match Time", 0).withPosition(6, 0).withSize(3, 1).getEntry();

    m_armVariables[0] =
        m_drivestationTab.add("Arm Position", 0).withPosition(0, 0).withSize(1, 1).getEntry();

    m_drivestationTab
        .add("Autonomous Chooser", m_autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);

    m_limelightVariables[0] =
        m_drivestationTab.add("Limelight Distance", 0).withPosition(7, 0).withSize(1, 1).getEntry();

    try {
      LLFeed = new HttpCamera("limelight", "http://10.39.28.92", HttpCameraKind.kMJPGStreamer);
      CameraServer.startAutomaticCapture(LLFeed);
      m_drivestationTab
          .add(LLFeed)
          .withPosition(5, 2)
          .withSize(3, 3)
          .withWidget(BuiltInWidgets.kCameraStream);
    } catch (VideoException e) {
    }
  }

  private void debugTab() {
    m_drivestationTab = Shuffleboard.getTab("Debug Tab");

    m_armVariables[1] =
        m_debugTab.add("Arm Voltage", 0).withPosition(0, 0).withSize(1, 1).getEntry();

    m_intakeVariables[0] =
        m_debugTab.add("Intake Wheel's Velocity", 0).withPosition(0, 1).withSize(1, 1).getEntry();

    m_intakeVariables[1] =
        m_debugTab
            .add("Up Down Encoder for Velocity", 0)
            .withPosition(0, 2)
            .withSize(1, 1)
            .getEntry();

    m_limelightVariables[1] =
        m_debugTab.add("Limelight ID", 0).withPosition(0, 3).withSize(1, 1).getEntry();

    m_limelightVariables[2] =
        m_debugTab.add("Limelight Tx", 0).withPosition(1, 3).withSize(1, 1).getEntry();

    m_limelightVariables[3] =
        m_debugTab.add("Limelight Ty", 0).withPosition(2, 3).withSize(1, 1).getEntry();

    m_limelightVariables[4] =
        m_debugTab.add("Limelight Tv", 0).withPosition(3, 3).withSize(1, 1).getEntry();
  }

  public void setUpSelector() {
    m_autoChooser.addOption("Auto 1", new ExampleCommand());
    m_autoChooser.addOption("Auto 2", new Autos());
  }

  public Command getAutoSelected() {
    return m_autoChooser.getSelected();
  }
}
