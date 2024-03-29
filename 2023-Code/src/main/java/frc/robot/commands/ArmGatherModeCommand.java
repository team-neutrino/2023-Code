// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.util.ViennaPIDController;

public class ArmGatherModeCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private TelescopeSubsystem m_telescopeSubsystem;
  private ViennaPIDController m_pidController;

  public ArmGatherModeCommand(
      SubsystemContainer p_subsystemContainer, ViennaPIDController p_pidController) {
    m_armSubsystem = p_subsystemContainer.getArmSubsystem();
    m_scoringSubsystem = p_subsystemContainer.getScoringSubsystem();
    m_intakeSubsystem = p_subsystemContainer.getIntakeSubsystem();
    m_telescopeSubsystem = p_subsystemContainer.getTelescopeSubsystem();
    m_pidController = p_pidController;
    addRequirements(m_armSubsystem, m_scoringSubsystem, m_intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intakeSubsystem.setIntakeDown();
    m_armSubsystem.smartArmSet(
        m_pidController.armRun(
            m_armSubsystem.getAbsoluteArmPosition(),
            ArmConstants.ARM_FRONTMOST,
            m_telescopeSubsystem.getTelescopingExtension()));

    if (m_armSubsystem.getAbsoluteArmPosition() >= ArmConstants.GATHER_POSITION) {
      if (m_intakeSubsystem.isIntakeDown()) {
        m_intakeSubsystem.unsqueeze();
      }
      m_scoringSubsystem.closeScoring();
    } else {
      m_scoringSubsystem.openScoring();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
