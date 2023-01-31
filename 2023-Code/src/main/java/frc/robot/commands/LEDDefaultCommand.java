package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ScoringSubsystem;

public class LEDDefaultCommand extends CommandBase {

  LEDSubsystem m_LedSubsystem;
  ScoringSubsystem m_ScoringSubsystem;
  private int m_option;
  private boolean m_beamBreak;

  public LEDDefaultCommand(LEDSubsystem p_LEDSubsystem, int p_option, boolean p_beamBreak) {
    m_LedSubsystem = p_LEDSubsystem;
    m_option = p_option;
    m_beamBreak = p_beamBreak;
    addRequirements(m_LedSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_option == 0) m_LedSubsystem.setToOrange();
    else if (m_option == 1) m_LedSubsystem.setToYellow();
    else if (m_option == 2) m_LedSubsystem.setToPurple();

    if (!m_beamBreak) end(m_beamBreak);
  }

  @Override
  public void end(boolean interrupted) {
    m_LedSubsystem.setToOrange();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
