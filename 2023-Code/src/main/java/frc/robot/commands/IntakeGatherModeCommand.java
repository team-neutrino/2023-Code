// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.IntakeManager;

/** This command will transfer game piece from intake to arm */
public class IntakeGatherModeCommand extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;
  private ArmSubsystem m_armSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private IntakeManager m_intakeManager;

  public IntakeGatherModeCommand(
      IntakeSubsystem p_intakeSubsystem,
      ArmSubsystem p_armSubsystem,
      ScoringSubsystem p_scoringSubsystem,
      IntakeManager p_intakeManager) {
    m_intakeSubsystem = p_intakeSubsystem;
    m_armSubsystem = p_armSubsystem;
    m_scoringSubsystem = p_scoringSubsystem;
    m_intakeManager = p_intakeManager;
    addRequirements(m_intakeSubsystem, m_armSubsystem, m_scoringSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_intakeManager.managerApproved()) {
      m_intakeSubsystem.setIntakeDown();

      if (!m_intakeSubsystem.detectedGamePiece() && m_intakeSubsystem.isIntakeDown()) {
        m_intakeSubsystem.unsqueeze();
        m_intakeSubsystem.runIntake();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return m_intakeSubsystem.detectedGamePiece() && m_intakeSubsystem.isIntakeDown();
  }
}
