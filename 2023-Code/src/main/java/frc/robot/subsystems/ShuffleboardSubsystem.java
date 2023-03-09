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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ExampleCommand;
import frc.robot.util.DriverStationInfo;
import frc.robot.util.IntakeManager;
import frc.robot.util.SavePoseCommand;
import frc.robot.util.ViennaPIDController;
import frc.robot.commands.autonomous.ScoreThenMove;

import java.io.IOException;
import frc.robot.commands.autonomous.ScoreMobilityThenBalance;

public class ShuffleboardSubsystem extends SubsystemBase {
  private ShuffleboardTab m_drivestationTab;
  private GenericEntry m_driveTrainVariables[] = new GenericEntry[11];
  private GenericEntry m_limelightVariables[] = new GenericEntry[8];
  private GenericEntry m_driverStationInfoVariables[] = new GenericEntry[9];
  private GenericEntry m_scoringVariables[] = new GenericEntry[11];
  private GenericEntry m_armVariables[] = new GenericEntry[1];
  private GenericEntry m_intakeVariables[] = new GenericEntry[4];
  private GenericEntry m_LEDVariables[] = new GenericEntry[4];
  private SendableChooser<Command> m_autonSelector = new SendableChooser<>();

  public SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  private DriveTrainSubsystem m_driveTrainSubsystem;
  private ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private LimelightSubsystem m_limelight;
  private ScoringSubsystem m_scoring;
  private DriverStationInfo m_driverStationInfo;
  private LEDSubsystem m_LED;
  private ArmSubsystem m_arm;
  private HttpCamera LLFeed;
  private IntakeSubsystem m_intake;
  private ViennaPIDController m_pidController;
  private IntakeManager m_intakeManager;

  public ShuffleboardSubsystem(
      DriverStationInfo p_driverStationInfo,
      DriveTrainSubsystem p_driveTrainSubsystem,
      ScoringSubsystem p_scoring,
      LimelightSubsystem p_limelight,
      ArmSubsystem p_arm,
      IntakeSubsystem p_intake,
      LEDSubsystem p_LED,
      ViennaPIDController p_pidController,
      IntakeManager p_intakeManager) {

    m_driveTrainSubsystem = p_driveTrainSubsystem;
    // m_limelight = p_limelight;
    m_scoring = p_scoring;
    m_arm = p_arm;
    m_intake = p_intake;
    m_LED = p_LED;
    m_pidController = p_pidController;
    m_intakeManager = p_intakeManager;
    m_driverStationInfo = p_driverStationInfo;
    m_drivestationTab = Shuffleboard.getTab("Driverstation Tab");

    setUpSelector();
    driveStationTab();
    setCommandButtons(); // This is SmartDashboard. Find way to make it not.
  }

  public ShuffleboardSubsystem() {}

  @Override
  public void periodic() {
    m_driveTrainVariables[0].setDouble(m_driveTrainSubsystem.getR1Pos());
    m_driveTrainVariables[1].setDouble(m_driveTrainSubsystem.getR2Pos());
    m_driveTrainVariables[2].setDouble(m_driveTrainSubsystem.getL1Pos());
    m_driveTrainVariables[3].setDouble(m_driveTrainSubsystem.getL2Pos());
    m_driveTrainVariables[4].setDouble(m_driveTrainSubsystem.getR1Vel());
    m_driveTrainVariables[5].setDouble(m_driveTrainSubsystem.getR2Vel());
    m_driveTrainVariables[6].setDouble(m_driveTrainSubsystem.getL1Vel());
    m_driveTrainVariables[7].setDouble(m_driveTrainSubsystem.getL2Vel());
    m_driveTrainVariables[8].setDouble(m_driveTrainSubsystem.getPitch());
    m_driveTrainVariables[9].setDouble(m_driveTrainSubsystem.getRoll());
    m_driveTrainVariables[10].setDouble(m_driveTrainSubsystem.getYaw());

    m_scoringVariables[0].setBoolean(m_scoring.getSolenoidValue());

    m_LEDVariables[0].setString(m_LED.getColor().toString());

    m_driverStationInfoVariables[0].setDouble(m_driverStationInfo.getMatchTime());

    m_armVariables[0].setDouble(m_arm.getAbsolutePosition());
  }

  private void driveStationTab() {
    m_drivestationTab = Shuffleboard.getTab("Drivestation Tab123");

    m_LEDVariables[0] =
        m_drivestationTab.add("LED Color", "Off").withPosition(8, 1).withSize(2, 1).getEntry();

    m_driveTrainVariables[0] =
        m_drivestationTab.add("RMotor 1 Pos", 0).withPosition(0, 0).withSize(1, 1).getEntry();

    m_driveTrainVariables[1] =
        m_drivestationTab.add("RMotor 2 Pos", 0).withPosition(1, 0).withSize(1, 1).getEntry();

    m_driveTrainVariables[2] =
        m_drivestationTab.add("LMotor 1 Pos", 0).withPosition(0, 1).withSize(1, 1).getEntry();

    m_driveTrainVariables[3] =
        m_drivestationTab.add("LMotor 2 Pos", 0).withPosition(1, 1).withSize(1, 1).getEntry();

    m_driveTrainVariables[4] =
        m_drivestationTab.add("RMotor 1 Vel", 0).withPosition(0, 2).withSize(1, 1).getEntry();

    m_driveTrainVariables[5] =
        m_drivestationTab.add("RMotor 2 Vel", 0).withPosition(1, 2).withSize(1, 1).getEntry();

    m_driveTrainVariables[6] =
        m_drivestationTab.add("LMotor 1 Vel", 0).withPosition(0, 3).withSize(1, 1).getEntry();

    m_driveTrainVariables[7] =
        m_drivestationTab.add("LMotor 2 Vel", 0).withPosition(1, 3).withSize(1, 1).getEntry();

    m_driveTrainVariables[8] =
        m_drivestationTab.add("NavX Pitch", 0).withPosition(2, 2).withSize(1, 1).getEntry();

    m_driveTrainVariables[9] =
        m_drivestationTab.add("NavX Roll", 0).withPosition(2, 3).withSize(1, 1).getEntry();

    m_driveTrainVariables[10] =
        m_drivestationTab.add("NavX Yaw", 0).withPosition(2, 4).withSize(1, 1).getEntry();

    m_scoringVariables[0] =
        m_drivestationTab.add("Grabber Release", 0).withPosition(2, 0).withSize(1, 1).getEntry();

    m_scoringVariables[1] =
        m_drivestationTab.add("Intake Down ", 0).withPosition(2, 1).withSize(1, 1).getEntry();

    m_scoringVariables[3] =
        m_drivestationTab.add("Squeeze", 0).withPosition(3, 0).withSize(1, 1).getEntry();

    m_driverStationInfoVariables[0] =
        m_drivestationTab.add("Match Time", 0).withPosition(4, 0).withSize(4, 2).getEntry();

    m_driverStationInfoVariables[1] =
        m_drivestationTab
            .add("Game Piece", "No Piece")
            .withPosition(8, 2)
            .withSize(2, 2)
            .getEntry();

    m_armVariables[0] =
        m_drivestationTab.add("Arm Position", 0).withPosition(3, 2).withSize(1, 1).getEntry();

    m_drivestationTab
        .add("Autonomous Chooser", m_autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(8, 0)
        .withSize(2, 1);

    m_limelightVariables[0] =
        m_drivestationTab.add("Distance", 0).withPosition(3, 1).withSize(1, 1).getEntry();

    try {
      LLFeed = new HttpCamera("limelight", "http://10.39.28.92", HttpCameraKind.kMJPGStreamer);
      CameraServer.startAutomaticCapture(LLFeed);
      m_drivestationTab
          .add(LLFeed)
          .withPosition(4, 2)
          .withSize(4, 3)
          .withWidget(BuiltInWidgets.kCameraStream);
    } catch (VideoException e) {
    }
  }

  public void setUpSelector() {
    m_autoChooser.setDefaultOption("Default", new ScoreMobilityThenBalance(m_driveTrainSubsystem, m_pidController, m_arm, m_scoring, m_intake, null, m_LED));
    m_autoChooser.addOption("Balance Mobility", new ScoreMobilityThenBalance(m_driveTrainSubsystem, m_pidController, m_arm, m_scoring, m_intake, m_intakeManager, m_LED));
    m_autoChooser.addOption("Score then Move", new ScoreThenMove(m_driveTrainSubsystem, m_pidController, m_arm, m_scoring, m_intake, m_intakeManager, m_LED));
  }

  public Command getAutoSelected() {
    return m_autoChooser.getSelected();
  }

  public void setUpAutoSelector() {
    m_autonSelector.addOption("Example Auto 1", new ExampleCommand());
    m_autonSelector.addOption("Example Auto 2", new ExampleCommand());
    m_autonSelector.addOption("Example Auto 2", new ExampleCommand());
  }

  public Command getAuto() {
    return m_autonSelector.getSelected();
  }

  private void setCommandButtons() {
    String filename = SmartDashboard.getString("trajectoryInput filename", "testInput.txt");
    try {
      SmartDashboard.putData("Save Pose", new SavePoseCommand(m_driveTrainSubsystem, filename));
    } catch (IOException e) {
      System.out.println("File write error");
      e.printStackTrace();
    }
  }
}
