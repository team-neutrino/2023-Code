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

/** An example command that uses an example subsystem. */
public class IntakeGatherModeCommand extends CommandBase {
  IntakeSubsystem m_intakeSubsystem;
  ArmSubsystem m_armSubsystem;
  ScoringSubsystem m_scoringSubsystem;
  IntakeManager m_intakeManager;
  ViennaPIDController m_pidController;

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
      System.out.println("execute of IntakeGatherModeCommand");
      m_intakeSubsystem.stopIntake();
      //m_armSubsystem.setReference(Constants.ArmConstants.FORWARD_DOWN);
      m_armSubsystem.smartSet(m_pidController.run(m_armSubsystem.getAbsolutePosition(), Constants.ArmConstants.ARM_FRONTMOST));

      //Math.abs(Constants.ArmConstants.FORWARD_DOWN - m_armSubsystem.getPosition()) <= Constants.ArmConstants.ARM_DEADZONE
      if (m_armSubsystem.getAbsolutePosition() > ArmConstants.FORWARD_DOWN) {
        m_scoringSubsystem.closeScoring();
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
