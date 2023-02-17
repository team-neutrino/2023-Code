// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.ViennaPIDController;

public class ArmFeederIntakeCommand extends CommandBase {
  ArmSubsystem m_armSubsystem;
  ScoringSubsystem m_ScoringSubsystem;
  ViennaPIDController m_pidController;
  double voltage;

  /** Creates a new ArmFeederIntakeCommand. */
  public ArmFeederIntakeCommand(
      ArmSubsystem p_armSubsystem,
      ScoringSubsystem p_scoringSubsystem,
      ViennaPIDController p_pidController) {
    m_armSubsystem = p_armSubsystem;
    m_ScoringSubsystem = p_scoringSubsystem;
    m_pidController = p_pidController;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    voltage = m_pidController.run(m_armSubsystem.getAbsolutePosition(), ArmConstants.BACK_MID);
    m_armSubsystem.set(voltage);

    if (m_ScoringSubsystem.gamePieceDetected()) {
      m_ScoringSubsystem.closeScoring();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
