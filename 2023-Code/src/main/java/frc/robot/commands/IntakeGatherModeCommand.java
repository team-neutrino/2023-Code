// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.IntakeManager;
import frc.robot.util.ViennaPIDController;

/** This command will transfer game piece from intake to arm */
public class IntakeGatherModeCommand extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;
  private ArmSubsystem m_armSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private IntakeManager m_intakeManager;
  private ViennaPIDController m_pidController;
  private int counter = 0;

  public IntakeGatherModeCommand(
      IntakeSubsystem p_intakeSubsystem,
      ArmSubsystem p_armSubsystem,
      ScoringSubsystem p_scoringSubsystem,
      IntakeManager p_intakeManager,
      ViennaPIDController p_pidController) {
    m_intakeSubsystem = p_intakeSubsystem;
    m_armSubsystem = p_armSubsystem;
    m_scoringSubsystem = p_scoringSubsystem;
    m_intakeManager = p_intakeManager;
    m_pidController = p_pidController;
    addRequirements(m_intakeSubsystem, m_armSubsystem, m_scoringSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intakeSubsystem.unsqueeze();
    m_intakeSubsystem.setIntakeDown();
    m_intakeSubsystem.runIntake();

    if (m_intakeSubsystem.detectedGamePiece()) {
      m_intakeSubsystem.stopIntake();
      m_armSubsystem.smartSet(
          m_pidController.run(
              m_armSubsystem.getAbsolutePosition(), Constants.ArmConstants.ARM_FRONTMOST));
      counter++; 

      if (m_armSubsystem.getAbsolutePosition() > ArmConstants.FORWARD_DOWN && counter > 20) {
        m_scoringSubsystem.closeScoring();
        counter = 0;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
