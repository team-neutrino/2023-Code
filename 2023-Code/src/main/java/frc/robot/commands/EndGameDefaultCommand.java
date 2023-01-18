package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
    frontChecker();
    backChecker();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean shouldExtendBack() {
    return m_rightJoystick.getY() >= Constants.VariableConstants.DEADZONE
        && m_leftJoystick.getY() >= Constants.VariableConstants.DEADZONE
        && m_rightJoystick.getTrigger()
        && !m_endgame.getSolenoidvalueBack();
  }

  private boolean shouldExtendFront() {
    return m_rightJoystick.getY() <= -Constants.VariableConstants.DEADZONE
        && m_leftJoystick.getY() <= -Constants.VariableConstants.DEADZONE
        && m_rightJoystick.getTrigger()
        && !m_endgame.getSolenoidvalueFront();
  }

  private void frontChecker() {
    if (shouldExtendFront()) {
      m_endgame.toggleSolenoidFront();
    } else if (!m_rightJoystick.getTrigger() && m_endgame.getSolenoidvalueFront()) {
      m_endgame.toggleSolenoidFront();
    }
  }

  private void backChecker() {
    if (shouldExtendBack()) {
      m_endgame.toggleSolenoidBack();
    } else if (!m_rightJoystick.getTrigger() && m_endgame.getSolenoidvalueBack()) {
      m_endgame.toggleSolenoidBack();
    }
  }
}
