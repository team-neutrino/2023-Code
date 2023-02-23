// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class PlayerStationCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private Timer timer = new Timer();

  /** Creates a new PlayerStationCommand. */
  public PlayerStationCommand(ArmSubsystem p_armSubsystem, ScoringSubsystem p_scoringSubsystem, IntakeSubsystem p_intakeSubsystem) {
    m_scoringSubsystem = p_scoringSubsystem;
    m_armSubsystem = p_armSubsystem;
    m_intakeSubsystem = p_intakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(p_armSubsystem, m_scoringSubsystem, p_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setReference(Constants.ArmConstants.BACK_MID);
    timer.start();

    if (m_scoringSubsystem.detectedGamePiece()) {
      m_scoringSubsystem.closeScoring();
      m_armSubsystem.setReference(Constants.ArmConstants.FORWARD_MID);
    }

    if (timer.get() >= 1) {
      m_intakeSubsystem.unsqueeze();
    }

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
