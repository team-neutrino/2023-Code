// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemContainer;
import frc.robot.commands.autonomous.ScoreHighThenMove;
import frc.robot.commands.autonomous.ScoreMobilityThenBalance;
import frc.robot.commands.autonomous.ScoreThenMove;
import frc.robot.util.DriverStationInfo;
import frc.robot.util.IntakeManager;
import frc.robot.util.SavePoseCommand;
import frc.robot.util.ViennaPIDController;
import java.io.IOException;
import java.util.Map;

public class ShuffleboardSubsystem extends SubsystemBase {
  private ShuffleboardTab m_drivestationTab;
  private ShuffleboardTab m_troubleshootingTab;
  private GenericEntry m_drivetrainVariables[] = new GenericEntry[11];
  private GenericEntry m_limelightVariables[] = new GenericEntry[8];
  private GenericEntry m_driverStationInfoVariables[] = new GenericEntry[9];
  private GenericEntry m_scoringVariables[] = new GenericEntry[11];
  private GenericEntry m_armVariables[] = new GenericEntry[1];
  private GenericEntry m_intakeVariables[] = new GenericEntry[4];
  private GenericEntry m_LEDVariables[] = new GenericEntry[4];
  private SendableChooser<Command> m_autonSelector = new SendableChooser<>();

  public SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  private SubsystemContainer m_subsystemContainer;
  private DriveTrainSubsystem m_drivetrainSubsystem;
  // private LimelightSubsystem m_limelight;
  private ScoringSubsystem m_scoring;
  private DriverStationInfo m_driverStationInfo;
  private LEDSubsystem m_LED;
  private ArmSubsystem m_arm;
  private HttpCamera LLFeed;
  private IntakeSubsystem m_intake;
  private ViennaPIDController m_pidController;
  private IntakeManager m_intakeManager;
  private XboxController m_driverController;
  private TelescopeSubsystem m_telescopeSubsystem;

  public ShuffleboardSubsystem(
      SubsystemContainer p_subsystemContainer,
      DriverStationInfo p_driverStationInfo,
      ViennaPIDController p_pidController,
      IntakeManager p_intakeManager,
      XboxController p_driverController) {

    // m_limelight = p_limelight;
    m_drivetrainSubsystem = p_subsystemContainer.getDriveTrainSubsystem();
    m_subsystemContainer = p_subsystemContainer;
    m_scoring = p_subsystemContainer.getScoringSubsystem();
    m_arm = p_subsystemContainer.getArmSubsystem();
    m_intake = p_subsystemContainer.getIntakeSubsystem();
    m_LED = p_subsystemContainer.getLedSubsystem();
    m_pidController = p_pidController;
    m_intakeManager = p_intakeManager;
    m_driverStationInfo = p_driverStationInfo;
    m_driverController = p_driverController;
    m_telescopeSubsystem = p_subsystemContainer.getTelescopeSubsystem();
    m_drivestationTab = Shuffleboard.getTab("Driverstation Tab");
    m_troubleshootingTab = Shuffleboard.getTab("Troubleshooting Tab");

    setUpSelector();
    driveStationTab();
    troubleshootingTab();
    setCommandButtons(); // This is SmartDashboard. Find way to make it not.
  }

  public ShuffleboardSubsystem() {}

  @Override
  public void periodic() {
    m_drivetrainVariables[0].setDouble(m_drivetrainSubsystem.getR1Pos());
    m_drivetrainVariables[1].setDouble(m_drivetrainSubsystem.getR2Pos());
    m_drivetrainVariables[2].setDouble(m_drivetrainSubsystem.getL1Pos());
    m_drivetrainVariables[3].setDouble(m_drivetrainSubsystem.getL2Pos());
    m_drivetrainVariables[4].setDouble(m_drivetrainSubsystem.getR1Vel());
    m_drivetrainVariables[5].setDouble(m_drivetrainSubsystem.getR2Vel());
    m_drivetrainVariables[6].setDouble(m_drivetrainSubsystem.getL1Vel());
    m_drivetrainVariables[7].setDouble(m_drivetrainSubsystem.getL2Vel());
    m_drivetrainVariables[8].setDouble(m_drivetrainSubsystem.getPitch());
    m_drivetrainVariables[9].setDouble(m_drivetrainSubsystem.getRoll());
    m_drivetrainVariables[10].setDouble(m_drivetrainSubsystem.getYaw());

    m_scoringVariables[0].setBoolean(m_scoring.getSolenoidValue());

    m_LEDVariables[0].setString(m_LED.getColor().toString());

    m_driverStationInfoVariables[0].setDouble(m_driverStationInfo.getMatchTime());

    m_armVariables[0].setDouble(m_arm.getAbsoluteArmPosition());
  }

  private void driveStationTab() {
    m_drivestationTab = Shuffleboard.getTab("Drivestation Tab123");

    m_LEDVariables[0] =
        m_drivestationTab.add("LED Color", "Off").withPosition(0, 3).withSize(2, 1).getEntry();

    m_scoringVariables[0] =
        m_drivestationTab.add("Grabber Release", 0).withPosition(0, 0).withSize(1, 1).getEntry();

    m_scoringVariables[1] =
        m_drivestationTab.add("Intake Down ", 0).withPosition(0, 1).withSize(1, 1).getEntry();

    m_scoringVariables[3] =
        m_drivestationTab.add("Squeeze", 0).withPosition(1, 0).withSize(1, 1).getEntry();

    m_driverStationInfoVariables[0] =
        m_drivestationTab
            .add("Match Time", 0)
            .withPosition(2, 0)
            .withSize(3, 2)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 0, "max", 150))
            .getEntry();
    m_driverStationInfoVariables[1] =
        m_drivestationTab
            .add("Game Piece", "No Piece")
            .withPosition(0, 2)
            .withSize(2, 1)
            .getEntry();

    m_armVariables[0] =
        m_drivestationTab.add("Arm Position", 0).withPosition(1, 1).withSize(1, 1).getEntry();

    m_drivestationTab
        .add("Autonomous Chooser", m_autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(5, 0)
        .withSize(2, 1);
    

    try {
      LLFeed = new HttpCamera("limelight", "http://10.39.28.92", HttpCameraKind.kMJPGStreamer);
      CameraServer.startAutomaticCapture(LLFeed);
      m_drivestationTab
          .add(LLFeed)
          .withPosition(2, 2)
          .withSize(3, 2)
          .withWidget(BuiltInWidgets.kCameraStream);
    } catch (VideoException e) {
    }
  }

  private void troubleshootingTab() {
    m_troubleshootingTab = Shuffleboard.getTab("Troubleshooting Tab");

    m_drivetrainVariables[0] =
        m_troubleshootingTab.add("RMotor 1 Pos", 0).withPosition(0, 0).withSize(1, 1).getEntry();

    m_drivetrainVariables[1] =
        m_troubleshootingTab.add("RMotor 2 Pos", 0).withPosition(1, 0).withSize(1, 1).getEntry();

    m_drivetrainVariables[2] =
        m_troubleshootingTab.add("LMotor 1 Pos", 0).withPosition(0, 1).withSize(1, 1).getEntry();

    m_drivetrainVariables[3] =
        m_troubleshootingTab.add("LMotor 2 Pos", 0).withPosition(1, 1).withSize(1, 1).getEntry();

    m_drivetrainVariables[4] =
        m_troubleshootingTab.add("RMotor 1 Vel", 0).withPosition(0, 2).withSize(1, 1).getEntry();

    m_drivetrainVariables[5] =
        m_troubleshootingTab.add("RMotor 2 Vel", 0).withPosition(1, 2).withSize(1, 1).getEntry();

    m_drivetrainVariables[6] =
        m_troubleshootingTab.add("LMotor 1 Vel", 0).withPosition(0, 3).withSize(1, 1).getEntry();

    m_drivetrainVariables[7] =
        m_troubleshootingTab.add("LMotor 2 Vel", 0).withPosition(1, 3).withSize(1, 1).getEntry();

    m_drivetrainVariables[8] =
        m_troubleshootingTab.add("NavX Pitch", 0).withPosition(2, 0).withSize(1, 1).getEntry();

    m_drivetrainVariables[9] =
        m_troubleshootingTab.add("NavX Roll", 0).withPosition(2, 1).withSize(1, 1).getEntry();

    m_drivetrainVariables[10] =
        m_troubleshootingTab.add("NavX Yaw", 0).withPosition(2, 2).withSize(1, 1).getEntry();

    m_limelightVariables[0] =
        m_troubleshootingTab.add("Distance", 0).withPosition(2, 3).withSize(1, 1).getEntry();
  }

  public void setUpSelector() {
    m_autoChooser.setDefaultOption(
        "Default",
        new ScoreMobilityThenBalance(
            m_subsystemContainer, m_pidController, m_intakeManager, m_driverController));
    m_autoChooser.addOption(
        "Balance Mobility",
        new ScoreMobilityThenBalance(
            m_subsystemContainer, m_pidController, m_intakeManager, m_driverController));
    m_autoChooser.addOption(
        "Score Mid then Move",
        new ScoreThenMove(
            m_subsystemContainer, m_pidController, m_intakeManager, m_driverController));
    m_autoChooser.addOption(
        "Score High then Move",
        new ScoreHighThenMove(
            m_subsystemContainer, m_pidController, m_intakeManager, m_driverController));
  }

  public Command getAutoSelected() {
    return m_autoChooser.getSelected();
  }

  public Command getAuto() {
    return m_autonSelector.getSelected();
  }

  private void setCommandButtons() {
    String filename = SmartDashboard.getString("trajectoryInput filename", "testInput.txt");
    try {
      SmartDashboard.putData("Save Pose", new SavePoseCommand(m_drivetrainSubsystem, filename));
    } catch (IOException e) {
      System.out.println("File write error");
      e.printStackTrace();
    }
  }
}
