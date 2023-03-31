package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.DriverStationInfo;
import frc.robot.util.EnumConstants.LEDColor;

public class LEDCommand extends CommandBase {

  private LEDSubsystem m_ledSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private LEDColor m_colorMode;
  private boolean hasBroken = false;

  public LEDCommand(
      SubsystemContainer p_subsystemContainer,
      LEDColor p_colorMode,
      DriverStationInfo p_DriverStationInfo) {
    m_ledSubsystem = p_subsystemContainer.getLedSubsystem();
    m_scoringSubsystem = p_subsystemContainer.getScoringSubsystem();
    m_colorMode = p_colorMode;
    addRequirements(m_ledSubsystem);
  }

  @Override
  public void initialize() {
    if (m_colorMode == LEDColor.PURPLE) {
      m_ledSubsystem.setToPurple();
    }
    if (m_colorMode == LEDColor.YELLOW) {
      m_ledSubsystem.setToYellow();
    }
  }

  @Override
  public void execute() {
    if (m_colorMode == LEDColor.PURPLE) {
      m_ledSubsystem.setToPurple();
    }
    if (m_colorMode == LEDColor.YELLOW) {
      m_ledSubsystem.setToYellow();
    }
    // if (m_scoringSubsystem.detectedGamePiece()) {
    //   hasBroken = true;
    // }
    // if (!m_scoringSubsystem.detectedGamePiece() && hasBroken) {
    //   m_ledSubsystem.setToOrange();
    //   hasBroken = false;
    // }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (m_ledSubsystem.getColor().equals(LEDColor.ORANGE)) {
      return true;
    }
    return false;
  }
}
