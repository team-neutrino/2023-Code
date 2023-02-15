package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.DriverStationInfo;
import frc.robot.util.LEDColor;

public class LEDCommand extends CommandBase {

  private LEDSubsystem m_LedSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private LEDColor m_colorMode;
  private boolean m_hasBroken = false;

  public LEDCommand(
      LEDSubsystem p_LEDSubsystem,
      LEDColor p_colorMode,
      ScoringSubsystem p_scoringSubsystem,
      DriverStationInfo p_DriverStationInfo) {
    m_LedSubsystem = p_LEDSubsystem;
    m_colorMode = p_colorMode;
    m_scoringSubsystem = p_scoringSubsystem;
    addRequirements(m_LedSubsystem);
  }

  @Override
  public void initialize() {
    if (m_colorMode == LEDColor.PURPLE) {
      m_LedSubsystem.setToPurple();
    }
    if (m_colorMode == LEDColor.YELLOW) {
      m_LedSubsystem.setToYellow();
    }
  }

  @Override
  public void execute() {
    if (m_scoringSubsystem.isGamePiece()) {
      m_hasBroken = true;
    }
    if (!m_scoringSubsystem.isGamePiece() && m_hasBroken) {
      m_LedSubsystem.setToOrange();
      m_hasBroken = false;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (m_LedSubsystem.getColor().equals(LEDColor.ORANGE)) {
      return true;
    }
    return false;
  }
}
