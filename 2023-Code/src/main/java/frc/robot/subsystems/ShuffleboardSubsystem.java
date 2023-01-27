// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.DriverStationInfo;

public class ShuffleboardSubsystem extends SubsystemBase {
  private ShuffleboardTab m_drivestationTab;
  private GenericEntry m_driveTrainVariables[] = new GenericEntry[8];
  private GenericEntry m_limelightVariables[] = new GenericEntry[8];
  private GenericEntry m_driverStationInfoVariables[] = new GenericEntry[9];
  private GenericEntry m_scoringVariables[] = new GenericEntry[11];
  private GenericEntry m_armVariables[] = new GenericEntry[1];

  private DriveTrainSubsystem m_driveTrain;
  private LimelightSubsystem m_limelight;
  private ScoringSubsystem m_scoring;
  private EndGameSubsystem m_endGame;
  private DriverStationInfo m_driverStationInfo;
  private ArmSubsystem m_arm;

  public ShuffleboardSubsystem(
      DriverStationInfo p_driverStationInfo,
      DriveTrainSubsystem p_driveTrain,
      ScoringSubsystem p_scoring,
      LimelightSubsystem p_limelight,
      ArmSubsystem p_arm) {

    m_driveTrain = p_driveTrain;
    m_limelight = p_limelight;
    m_scoring = p_scoring;
    m_arm = p_arm;

    m_driverStationInfo = p_driverStationInfo;
    m_drivestationTab = Shuffleboard.getTab("Driverstation Tab");

    driveStationTab();
  }

  @Override
  public void periodic() {
    m_driveTrainVariables[0].setDouble(m_driveTrain.getR1Pos());
    m_driveTrainVariables[1].setDouble(m_driveTrain.getR2Pos());
    m_driveTrainVariables[2].setDouble(m_driveTrain.getL1Pos());
    m_driveTrainVariables[3].setDouble(m_driveTrain.getL2Pos());
    //m_driveTrainVariables[4].setDouble(m_driveTrain.getR1Vel());
    m_driveTrainVariables[5].setDouble(m_driveTrain.getR2Vel());
    m_driveTrainVariables[6].setDouble(m_driveTrain.getL1Vel());
    m_driveTrainVariables[7].setDouble(m_driveTrain.getL2Vel());
    
    m_driverStationInfoVariables[0].setDouble(m_driverStationInfo.getMatchTime());

    m_armVariables[0].setDouble(m_arm.getPosition());
  }

  private void driveStationTab() {
    m_drivestationTab = Shuffleboard.getTab("Drivestation Tab");

    m_driveTrainVariables[0] =
        m_drivestationTab
            .add("Right Motor 1 Velocity", 0)
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

    m_driveTrainVariables[5] =
        m_drivestationTab
            .add("Right Motor 2 Velocity", 0)
            .withPosition(0, 4)
            .withSize(1, 1)
            .getEntry();

    m_driveTrainVariables[6] =
        m_drivestationTab
            .add("Left Motor 1 Velocity", 0)
            .withPosition(0, 5)
            .withSize(1, 1)
            .getEntry();

    m_driveTrainVariables[7] =
        m_drivestationTab
            .add("Left Motor 2 Velocity", 0)
            .withPosition(0, 6)
            .withSize(1, 1)
            .getEntry();

    m_limelightVariables[0] =
        m_drivestationTab.add("Distance", 0).withPosition(1, 0).withSize(1, 1).getEntry();

    m_driverStationInfoVariables[0] =
        m_drivestationTab.add("Match Time", 0).withPosition(2, 0).withSize(3, 1).getEntry();

    m_armVariables[0] =
        m_drivestationTab.add("Arm Position", 0).withPosition(3, 0).withSize(1, 1).getEntry();
  }
}
