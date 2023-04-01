// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.IntakeManager;

/** Command will intake the game piece without transfering it to the arm */
public class IntakeHybridModeCommand extends CommandBase {
  IntakeSubsystem m_intakeSubsystem;
  IntakeManager m_intakeManager;

  public IntakeHybridModeCommand(
      SubsystemContainer p_subsystemContainer, IntakeManager p_intakeManager) {
    m_intakeSubsystem = p_subsystemContainer.getIntakeSubsystem();
    m_intakeManager = p_intakeManager;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_intakeManager.managerApproved()) {
      m_intakeSubsystem.unsqueeze();
      m_intakeSubsystem.setIntakeDown();
      m_intakeSubsystem.runIntake();
    }

    // TODO move this logic into isFinished because it's duplicate code from intakeDefaultCommand
    if (m_intakeSubsystem.detectedGamePiece()) {
      m_intakeSubsystem.stopIntake();
      m_intakeSubsystem.squeeze();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.resetDebouncer();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
