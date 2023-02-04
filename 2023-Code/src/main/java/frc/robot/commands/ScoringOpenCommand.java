// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ScoringSubsystem;

public class ScoringOpenCommand extends CommandBase {
  private ScoringSubsystem m_scoringSubsystem;

  /** Creates a new ScoringCommand. */
  public ScoringOpenCommand(ScoringSubsystem p_scoringSubsystem) {
    m_scoringSubsystem = p_scoringSubsystem;
    addRequirements(m_scoringSubsystem);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    m_scoringSubsystem.extendSolenoid();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
