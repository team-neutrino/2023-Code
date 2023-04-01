// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.IntakeManager;

/** This command will transfer game piece from intake to arm */
public class IntakeGatherModeCommand extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;
  private IntakeManager m_intakeManager;
  private boolean m_auton;

  public IntakeGatherModeCommand(
      SubsystemContainer p_subsystemContainer, IntakeManager p_intakeManager, boolean p_auton) {
    m_intakeSubsystem = p_subsystemContainer.getIntakeSubsystem();
    m_intakeManager = p_intakeManager;
    m_auton = p_auton;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if (m_auton) {
      m_intakeSubsystem.runIntake();
    }

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
    if (!m_auton) {
      m_intakeSubsystem.stopIntake();
    }
    m_intakeSubsystem.resetDebouncer();
    // TODO remove potentially, does the same thing as IntakeDefaultCommand
  }

  @Override
  public boolean isFinished() {
    return m_intakeSubsystem.detectedGamePiece() && m_intakeSubsystem.isIntakeDown();
  }
}
