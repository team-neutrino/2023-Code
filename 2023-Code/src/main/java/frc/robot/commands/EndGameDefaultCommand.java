package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndGameSubsystem;

public class EndGameDefaultCommand extends CommandBase {
  EndGameSubsystem m_endgame;

  public EndGameDefaultCommand(EndGameSubsystem p_endgame) {
    m_endgame = p_endgame;
    addRequirements(m_endgame);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
