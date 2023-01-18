package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndGameSubsystem;

public class EndGameDefaultCommand extends CommandBase {
  EndGameSubsystem m_endgame;
  Joystick m_rightJoystick;
  Joystick m_leftJoystick;

  public EndGameDefaultCommand(
      EndGameSubsystem p_endgame, Joystick p_rightJoystick, Joystick p_leftJoystick) {
    m_endgame = p_endgame;
    m_rightJoystick = p_rightJoystick;
    m_leftJoystick = p_leftJoystick;
    addRequirements(m_endgame);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    forwardChecker();
    backwardChecker();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean ShouldExtendBackward() {
    return m_rightJoystick.getY() >= 0.1
        && m_leftJoystick.getY() >= 0.1
        && m_rightJoystick.getTrigger()
        && !m_endgame.getSolenoidvalueBackward();
  }

  private boolean ShouldExtendForward() {
    return m_rightJoystick.getY() <= -0.1
        && m_leftJoystick.getY() <= -0.1
        && m_rightJoystick.getTrigger()
        && !m_endgame.getSolenoidvalueForward();
  }

  private void forwardChecker() {
    if (ShouldExtendForward()) {
      m_endgame.toggleSolenoidForward();
    } else if (!m_rightJoystick.getTrigger() && m_endgame.getSolenoidvalueForward()) {
      m_endgame.toggleSolenoidForward();
    }
  }

  private void backwardChecker() {
    if (ShouldExtendBackward()) {
      m_endgame.toggleSolenoidBackward();
    } else if (!m_rightJoystick.getTrigger() && m_endgame.getSolenoidvalueBackward()) {
      m_endgame.toggleSolenoidBackward();
    }
  }
}
