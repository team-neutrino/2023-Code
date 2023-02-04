package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ScoringSubsystem;

public class LEDDefaultCommand extends CommandBase {

  LEDSubsystem m_LedSubsystem;
  ScoringSubsystem m_ScoringSubsystem;
  private int m_option;
  private boolean m_beamBreak;

  public LEDDefaultCommand(
      LEDSubsystem p_LEDSubsystem, int p_option, ScoringSubsystem p_scoringSubsystem) {
    m_LedSubsystem = p_LEDSubsystem;
    m_option = p_option;
    m_ScoringSubsystem = p_scoringSubsystem;
    addRequirements(m_LedSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_option == 1 && m_ScoringSubsystem.getBeamBreak()) m_LedSubsystem.setToYellow();
    else if (m_option == 2 && m_ScoringSubsystem.getBeamBreak()) m_LedSubsystem.setToPurple();
    else if (!m_ScoringSubsystem.getBeamBreak()) m_LedSubsystem.setToOrange();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
