// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ScoringSubsystem;

public class ScoringDefaultCommand extends CommandBase {
  private ScoringSubsystem m_scoringSubsystem;
  /** Creates a new ScoringDefaultCommand. */
  public ScoringDefaultCommand(ScoringSubsystem p_scoringSubsystem) {
    m_scoringSubsystem = p_scoringSubsystem;
    addRequirements(m_scoringSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_scoringSubsystem.setSolenoidExtend();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // m_scoringSubsystem.setSolenoidOff();
   System.out.println("Running DEFAULT command");
   m_scoringSubsystem.setSolenoidExtend();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
