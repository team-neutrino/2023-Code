// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.EndGameSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class ShuffleboardSubsystem extends SubsystemBase {
  private ShuffleboardTab m_drivestationTab;
  private NetworkTableEntry m_scoringVariables[] = new NetworkTableEntry[0];
  private NetworkTableEntry m_driveTrainVariables [] = new NetworkTableEntry[7];
  private NetworkTableEntry m_endGameVariables [] = new NetworkTableEntry[1];
  private NetworkTableEntry m_limelightVariables [] = new NetworkTableEntry [4];
  

  ScoringSubsystem m_scoring;
  DriveTrainSubsystem m_driveTrain;
  EndGameSubsystem m_endGame;
  LimelightSubsystem m_limelight;

  /** Creates a new ShuffleboardSubsystem. */
  public ShuffleboardSubsystem(ScoringSubsystem p_scoring, DriveTrainSubsystem p_driveTrain, EndGameSubsystem p_endGame, LimelightSubsystem p_limelight) {
    m_scoring = p_scoring;
    m_driveTrain = p_driveTrain;
    m_endGame = p_endGame;
    m_limelight = p_limelight;
    m_drivestationTab = Shuffleboard.getTab("driverstation tab");
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
  m_endGameVariables[1].sem_endGame.getSolenoidvalueBack());
  m_limelightVariables[0].setBoolean(m_limelight.getTv());
  m_limelightVariables[1].setDouble(m_limelight.getID());
  m_limelightVariables[2].setDouble(m_limelight.getTx());
  m_limelightVariables[3].setDouble(m_limelight.getTy());
  m_limelightVariables[4].setDouble(m_limelight.getDistance());

  }

  private void driveStaionTab() {
    m_drivestationTab = Shuffleboard.getTab("Drivestation Tab");
    m_endGameVariables[1] =
    m_drivestationTab
        .add("Index Beambreak", false)
        .withPosition(6, 4)
        .withSize(1, 1)
        .getEntry();
  }
}