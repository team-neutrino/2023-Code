// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.IntakeManager;

public class IntakeCommand extends CommandBase {

  private IntakeSubsystem m_intakeSubsystem;
  private IntakeManager m_intakeManager;
  private Timer timer;
  private double m_time = 60 * 60 * 24;

  public IntakeCommand(IntakeSubsystem p_intakeSubsystem, IntakeManager p_intakeManager) {
    m_intakeSubsystem = p_intakeSubsystem;
    m_intakeManager = p_intakeManager;

    addRequirements(m_intakeSubsystem);
  }

  public IntakeCommand(
      IntakeSubsystem p_intakeSubsystem, IntakeManager p_intakeManager, double p_time) {
    m_intakeSubsystem = p_intakeSubsystem;
    m_intakeManager = p_intakeManager;
    timer = new Timer();
    m_time = p_time;

    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    if (m_intakeManager.managerApproved()) {
      if (!m_intakeSubsystem.detectedGamePiece()) {
        m_intakeSubsystem.runIntake();
        m_intakeManager.setIntakeDownWithArmCheck();
        m_intakeSubsystem.unsqueeze();
      } else {
        m_intakeSubsystem.squeeze();
        m_intakeSubsystem.stopIntake();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
  }

  @Override
  public boolean isFinished() {
    if (timer.get() >= m_time) {
      return true;
    }
    return false;
  }
}
