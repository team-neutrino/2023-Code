// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.IntakeManager;

public class ScoringOpenCommand extends CommandBase {
  private ScoringSubsystem m_scoringSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private IntakeManager m_intakeManager;

  public ScoringOpenCommand(
      ScoringSubsystem p_scoringSubsystem,
      IntakeSubsystem p_intakeSubsystem,
      IntakeManager p_intakeManager) {
    m_scoringSubsystem = p_scoringSubsystem;
    m_intakeSubsystem = p_intakeSubsystem;
    m_intakeManager = p_intakeManager;
    addRequirements(m_scoringSubsystem, m_intakeSubsystem);
  }

  public ScoringOpenCommand(
      ScoringSubsystem p_scoringSubsystem,
      IntakeSubsystem p_intakeSubsystem,
      IntakeManager p_intakeManager,
      double p_time) {
    m_scoringSubsystem = p_scoringSubsystem;
    m_intakeSubsystem = p_intakeSubsystem;
    m_intakeManager = p_intakeManager;

    addRequirements(p_scoringSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intakeManager.managerApproved()) {
      m_scoringSubsystem.openScoring();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
