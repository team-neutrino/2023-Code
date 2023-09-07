// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.IntakeManager;

/**
 * This default command serves the sole purpose of ensuring that when the button is NOT pressed, the
 * intake is up and stopped.
 */
public class IntakeDefaultCommand extends CommandBase {
  IntakeSubsystem m_intakeSubsystem;
  IntakeManager m_intakeManager;
  SubsystemContainer m_subsystemContainer;
  boolean m_safetystate;

  public IntakeDefaultCommand(
      SubsystemContainer p_subsystemContainer, IntakeManager p_intakeManager) {

    m_subsystemContainer = p_subsystemContainer;
    m_intakeSubsystem = p_subsystemContainer.getIntakeSubsystem();
    m_intakeManager = p_intakeManager;

    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystemContainer.getIntakeSubsystem().stopIntake();

    if (m_intakeManager.managerApproved()) {
      m_intakeManager.setIntakeUpWithArmCheck();
      
      if(m_safetystate) {

      }
      else {
        
        if (!m_subsystemContainer.getIntakeSubsystem().unDebouncedDetectedGamePiece()) {
        m_subsystemContainer.getIntakeSubsystem().unsqueeze();
      } else {
        m_subsystemContainer.getIntakeSubsystem().squeeze();
      }
    }
  }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
