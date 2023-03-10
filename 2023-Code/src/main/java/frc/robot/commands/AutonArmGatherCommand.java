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

public class AutonArmGatherCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private ViennaPIDController m_pidController;
  private boolean detective;

  public AutonArmGatherCommand(
      ArmSubsystem p_armSubsystem,
      ScoringSubsystem p_scoringSubsystem,
      IntakeSubsystem p_intakeSubsystem,
      ViennaPIDController p_pidController) {
    m_armSubsystem = p_armSubsystem;
    m_scoringSubsystem = p_scoringSubsystem;
    m_intakeSubsystem = p_intakeSubsystem;
    m_pidController = p_pidController;
    detective = false;

    addRequirements(m_armSubsystem, m_scoringSubsystem, m_intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intakeSubsystem.setIntakeDown();

    if (detective == false) {
      if (m_intakeSubsystem.detectedGamePiece()) {
        detective = true;
      }
      m_intakeSubsystem.runIntake();
      m_scoringSubsystem.openScoring();
      m_armSubsystem.smartArmSet(
          m_pidController.run(m_armSubsystem.getAbsoluteArmPosition(), ArmConstants.ARM_FRONTMOST));
    } else {
      m_intakeSubsystem.stopIntake();
      m_scoringSubsystem.closeScoring();
      m_armSubsystem.smartArmSet(
          m_pidController.run(m_armSubsystem.getAbsoluteArmPosition(), ArmConstants.FORWARD_MID));
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setIntakeUp();
  }

  @Override
  public boolean isFinished() {
    if (m_intakeSubsystem.detectedGamePiece()
        && Math.abs(m_armSubsystem.getAbsoluteArmPosition() - ArmConstants.FORWARD_MID) < 1) {
      return true;
    }
    return false;
  }
}
