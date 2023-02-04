package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ScoringSubsystem;

public class LEDCommand extends CommandBase {

  LEDSubsystem m_LedSubsystem;
  ScoringSubsystem m_ScoringSubsystem;
  private int m_option;
  private boolean m_hasBroken = false;

  public LEDCommand(
      LEDSubsystem p_LEDSubsystem, int p_option, ScoringSubsystem p_scoringSubsystem) {
    m_LedSubsystem = p_LEDSubsystem;
    m_option = p_option;
    m_ScoringSubsystem = p_scoringSubsystem;
    addRequirements(m_LedSubsystem, m_ScoringSubsystem);
  }

  @Override
  public void initialize() {
    if (m_option == 1) {
      m_LedSubsystem.setToPurple();
    }
    if (m_option == 2) {
    m_LedSubsystem.setToYellow();
    }
  }

  @Override
  public void execute() {
    if (!m_ScoringSubsystem.getBeamBreak()) { //broken
      m_hasBroken = true;
    }
    if (m_ScoringSubsystem.getBeamBreak() && m_hasBroken) { // beambreak not broken and hasBroken = true
      m_LedSubsystem.setToOrange();
      m_hasBroken = false;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (m_LedSubsystem.getColor().equals("Orange")) {
      System.out.println("finshed");
      return true;
    }
    return false;
  }
}
