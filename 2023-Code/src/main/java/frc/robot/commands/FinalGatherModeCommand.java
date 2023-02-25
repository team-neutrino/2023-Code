// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.ViennaPIDController;

public class FinalGatherModeCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private ViennaPIDController m_pidController;

  /** Creates a new FinalGatherModeCommand. */
  public FinalGatherModeCommand(ArmSubsystem p_armSubsystem, IntakeSubsystem p_intakeSubsystem, ScoringSubsystem p_scoringSubsystem, ViennaPIDController p_pidController) {
    m_armSubsystem = p_armSubsystem;
    m_intakeSubsystem = p_intakeSubsystem;
    m_scoringSubsystem = p_scoringSubsystem;
    m_pidController = p_pidController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.smartSet(
      m_pidController.run(m_armSubsystem.getAbsolutePosition(), ArmConstants.FORWARD_MID));
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
