// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
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
     SubsystemContainer p_subsystemContainer,
     ViennaPIDController p_pidController) {
    m_armSubsystem = p_subsystemContainer.getArmSubsystem();
    m_scoringSubsystem = p_subsystemContainer.getScoringSubsystem();
    m_intakeSubsystem = p_subsystemContainer.getIntakeSubsystem();
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
      m_armSubsystem.smartSet(
          m_pidController.run(m_armSubsystem.getAbsolutePosition(), ArmConstants.ARM_FRONTMOST));
    } else {
      m_intakeSubsystem.stopIntake();
      m_scoringSubsystem.closeScoring();
      m_armSubsystem.smartSet(
          m_pidController.run(m_armSubsystem.getAbsolutePosition(), ArmConstants.FORWARD_MID));
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setIntakeUp();
  }

  @Override
  public boolean isFinished() {
    if (m_intakeSubsystem.detectedGamePiece()
        && Math.abs(m_armSubsystem.getAbsolutePosition() - ArmConstants.FORWARD_MID) < 1) {
      return true;
    }
    return false;
  }
}
