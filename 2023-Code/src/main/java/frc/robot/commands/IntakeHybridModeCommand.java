// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.IntakeManager;

/** An example command that uses an example subsystem. */
public class IntakeHybridModeCommand extends CommandBase {
  IntakeSubsystem m_intakeSubsystem;
  ArmSubsystem m_armSubsystem;
  ScoringSubsystem m_scoringSubsystem;
  IntakeManager m_intakeManager;

  
  public IntakeHybridModeCommand(IntakeSubsystem p_intakeSubsystem , ArmSubsystem p_armSubsystem , ScoringSubsystem p_scoringSubsystem, IntakeManager p_intakeManager) {
    m_intakeSubsystem = p_intakeSubsystem;
    m_armSubsystem = p_armSubsystem;
    m_scoringSubsystem = p_scoringSubsystem;
    m_intakeManager = p_intakeManager;
    addRequirements(m_intakeSubsystem, m_armSubsystem, m_scoringSubsystem);
  }

  @Override
  public void initialize() {
    if (m_intakeSubsystem.detectedGamePiece()) {
      m_intakeSubsystem.stopIntake();
      m_intakeSubsystem.squeeze();
      m_intakeManager.setIntakeUpWithArmCheck();
      m_armSubsystem.setReference(Constants.ArmConstants.FORWARD_DOWN);
  }
}

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
