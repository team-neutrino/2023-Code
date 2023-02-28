// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.IntakeManager;

public class ScoringOpenCommand extends CommandBase {
  private ScoringSubsystem m_scoringSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private IntakeManager m_intakeManager;
  private Timer timer;
  private double m_time = 60 * 60 * 24;

  public ScoringOpenCommand(ScoringSubsystem p_scoringSubsystem, IntakeSubsystem p_intakeSubsystem, IntakeManager p_intakeManager) {
    m_scoringSubsystem = p_scoringSubsystem;
    m_intakeSubsystem = p_intakeSubsystem;
    m_intakeManager = p_intakeManager;
    timer = new Timer();
    addRequirements(m_scoringSubsystem, m_intakeSubsystem);
  }

  public ScoringOpenCommand(ScoringSubsystem p_scoringSubsystem, double p_time) {
    m_scoringSubsystem = p_scoringSubsystem;
    timer = new Timer();
    m_time = p_time;

    addRequirements(p_scoringSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_intakeManager.managerApproved()){
      m_scoringSubsystem.openScoring();
      m_intakeManager.setIntakeDownWithArmCheck();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() > m_time) {
      return true;
    }
    return false;
  }
}
