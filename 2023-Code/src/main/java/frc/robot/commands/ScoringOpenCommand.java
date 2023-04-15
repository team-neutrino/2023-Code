// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.IntakeManager;

public class ScoringOpenCommand extends CommandBase {
  private ScoringSubsystem m_scoringSubsystem;
  private IntakeManager m_intakeManager;

  public ScoringOpenCommand(
      SubsystemContainer p_subsystemContainer, IntakeManager p_intakeManager) {
    m_scoringSubsystem = p_subsystemContainer.getScoringSubsystem();
    m_intakeManager = p_intakeManager;
    addRequirements(m_scoringSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_intakeManager.managerApproved()) {
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
