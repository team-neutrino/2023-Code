// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.IntakeManager;

public class IntakeCommand extends CommandBase {

  IntakeSubsystem m_intakeSubsystem;
  IntakeManager m_intakeManager;

  public IntakeCommand(IntakeSubsystem p_intakeSubsystem, IntakeManager p_intakeManager) {
    m_intakeSubsystem = p_intakeSubsystem;
    m_intakeManager = p_intakeManager;

    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_intakeManager.managerApproved()) {
      m_intakeSubsystem.runIntake();
      m_intakeManager.setIntakeDownWithArmCheck();
      m_intakeSubsystem.unsqueeze();
      if (m_intakeSubsystem.detectedGamePiece()) {
        m_intakeSubsystem.squeeze();
        m_intakeSubsystem.stopIntake();
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
