// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndGameSubsystem;

public class EndGameToggleCommand extends CommandBase {
  EndGameSubsystem m_endgame;
  /** Creates a new EndGame_toggle. */
  public EndGameToggleCommand(EndGameSubsystem p_endGame) {
    m_endgame = p_endGame;
    addRequirements(m_endgame);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_endgame.toggleSolenoidRight();
    m_endgame.toggleSolenoidLeft();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
