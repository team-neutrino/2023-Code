// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.IntakeManager;
import frc.robot.util.ViennaPIDController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.Constants.*;

/** This command will transfer game piece from intake to arm */
public class IntakeGatherModeCommand extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;
  private ArmSubsystem m_armSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private ViennaPIDController m_pidController;
  private IntakeManager m_intakeManager;
  private boolean m_auton;

  public IntakeGatherModeCommand(
      IntakeSubsystem p_intakeSubsystem, IntakeManager p_intakeManager, ArmSubsystem p_armSubsystem, ScoringSubsystem p_scoringSubystem, ViennaPIDController p_pidController, boolean p_auton) {
    m_intakeSubsystem = p_intakeSubsystem;
    m_intakeManager = p_intakeManager;
    m_armSubsystem = p_armSubsystem;
    m_scoringSubsystem = p_scoringSubystem;
    m_pidController = p_pidController;
    m_auton = p_auton;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if (m_auton) {
      m_intakeSubsystem.runIntake();
    }

    if (m_intakeManager.managerApproved()) {
      m_intakeSubsystem.setIntakeDown();

      if (!m_intakeSubsystem.detectedGamePiece() && m_intakeSubsystem.isIntakeDown()) {
        m_intakeSubsystem.unsqueeze();
        m_intakeSubsystem.runIntake();
      }
      if (m_intakeSubsystem.detectedGamePiece()) {
        m_armSubsystem.smartSet(
          m_pidController.run(m_armSubsystem.getAbsolutePosition(), ArmConstants.ARM_FRONTMOST));
  
        if (m_armSubsystem.getAbsolutePosition() >= ArmConstants.GATHER_POSITION) {

          if (m_intakeSubsystem.isIntakeDown()) {
            m_intakeSubsystem.unsqueeze();
          }

          m_scoringSubsystem.closeScoring();
        }
        else {
          m_scoringSubsystem.openScoring();
          }
        }
      }
    }

  @Override
  public void end(boolean interrupted) {
    if (!m_auton) {
      m_intakeSubsystem.stopIntake();
    }
    // TODO remove potentially, does the same thing as IntakeDefaultCommand
  }

  @Override
  public boolean isFinished() {
    return m_intakeSubsystem.detectedGamePiece() && m_intakeSubsystem.isIntakeDown();
  }
}
