// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShuffleboardSubsystem extends SubsystemBase {
  private ShuffleboardTab m_drivestationTab;
  private GenericEntry m_scoringVariables[] = new GenericEntry[1];
  private GenericEntry m_driveTrainVariables[] = new GenericEntry[7];
  private GenericEntry m_endGameVariables[] = new GenericEntry[1];
  private GenericEntry m_limelightVariables[] = new GenericEntry[4];
  private GenericEntry m_driverStationInfo[] = new GenericEntry[2];
  private GenericEntry distanceEntry;

  ScoringSubsystem m_scoring;
  DriveTrainSubsystem m_driveTrain;
  EndGameSubsystem m_endGame;
  LimelightSubsystem m_limelight;

  /** Creates a new ShuffleboardSubsystem. */
  public ShuffleboardSubsystem(
      ScoringSubsystem p_scoring, DriveTrainSubsystem p_driveTrain, EndGameSubsystem p_endGame, LimelightSubsystem p_limelight) {
    m_scoring = p_scoring;
    m_driveTrain = p_driveTrain;
    m_endGame = p_endGame;
    m_limelight = p_limelight;
    m_drivestationTab = Shuffleboard.getTab("driverstation tab");

    //driveStaionTab();
  }

  @Override 
  public void periodic() {
   m_scoringVariables[0].setBoolean(m_scoring.getSolenoidValue());
    m_driveTrainVariables[0].setDouble(m_driveTrain.getR1Pos());
    m_driveTrainVariables[1].setDouble(m_driveTrain.getR2Pos());
    m_driveTrainVariables[2].setDouble(m_driveTrain.getL1Pos());
    m_driveTrainVariables[3].setDouble(m_driveTrain.getL2Pos());
    m_driveTrainVariables[4].setDouble(m_driveTrain.getR1Vel());
    m_driveTrainVariables[5].setDouble(m_driveTrain.getR2Vel());
    m_driveTrainVariables[6].setDouble(m_driveTrain.getL1Vel());
    m_driveTrainVariables[7].setDouble(m_driveTrain.getL2Vel());
    m_endGameVariables[1].setBoolean(m_endGame.getSolenoidvalueBack());
  }

  private void driveStaionTab() {
    m_drivestationTab = Shuffleboard.getTab("Drivestation Tab");

    m_endGameVariables[0] =
        m_drivestationTab.add("isBlue", false).withPosition(6, 0).withSize(1, 1).getEntry();

    m_limelightVariables[0] = 
        m_drivestationTab.add("Distance", 0).withPosition(7, 0).withSize(1, 1).getEntry();

    m_driverStationInfo[0] = 
        m_drivestationTab.add("Match Time", 0).withPosition(0,0).withSize(3,1).getEntry();

    m_driveTrainVariables[0] = 
      m_drivestationTab.add("Right Motor 1 Position", 0).withPosition(0,0).withSize(1,1).getEntry();
  
    m_driveTrainVariables[1] = 
      m_drivestationTab.add("Right Motor 2 Position", 0).withPosition(0,0).withSize(1,1).getEntry();
  }
}