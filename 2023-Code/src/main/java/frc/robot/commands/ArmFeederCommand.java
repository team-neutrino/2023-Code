// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.ViennaPIDController;

public class ArmFeederCommand extends CommandBase {
  ArmSubsystem m_armSubsystem;
  ScoringSubsystem m_scoringSubsystem;
  ViennaPIDController m_pidController;
  boolean beamBreakBroken;
  double voltage;

  /** Creates a new ArmFeederIntakeCommand. */
  public ArmFeederCommand(
      ArmSubsystem p_armSubsystem,
      ScoringSubsystem p_scoringSubsystem,
      ViennaPIDController p_pidController) {
    m_armSubsystem = p_armSubsystem;
    m_scoringSubsystem = p_scoringSubsystem;
    m_pidController = p_pidController;
    beamBreakBroken = false;
  }

  @Override
  public void initialize() {
    m_scoringSubsystem.openScoring();
  }

  @Override
  public void execute() {
    m_armSubsystem.smartSet(
        m_pidController.run(m_armSubsystem.getAbsolutePosition(), ArmConstants.FEEDER_POSITION));

    // if (m_scoringSubsystem.detectedGamePiece()) {
    //   beamBreakBroken = true;
    // }

    System.out.println(m_scoringSubsystem.detectedGamePiece());

    if (m_scoringSubsystem.detectedGamePiece()) {
      m_scoringSubsystem.closeScoring();
    } else {
      m_scoringSubsystem.openScoring();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
