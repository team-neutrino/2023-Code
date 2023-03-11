// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.IntakeManager;

/**
 * This default command serves the sole purpose of ensuring that when the button is NOT pressed, the
 * intake is up and stopped.
 */
public class IntakeDefaultCommand extends CommandBase {
  /** An object of the intake subsystem. */
  IntakeSubsystem m_intakeSubsystem;

  IntakeManager m_intakeManager;

  /** Constructor, creates a new IntakeDefaultCommand. */
  public IntakeDefaultCommand(
      SubsystemContainer p_subsystemContainer, IntakeManager p_inIntakeManager) {
    m_intakeSubsystem = p_subsystemContainer.getIntakeSubsystem();
    m_intakeManager = p_inIntakeManager;

    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {}

  /**
   * This command's purpose is to ensure that the intake is up and not running when the button isn't
   * pressed.
   */
  @Override
  public void execute() {
    m_intakeSubsystem.stopIntake();

    if (m_intakeManager.managerApproved()) {
      m_intakeManager.setIntakeUpWithArmCheck();

      // code for squeezing & unsqueezing is controlled by arm position due to problem with
      // intake squeezing ball right after arm gets ahold of it and partly pulling it out.
      if (!m_intakeSubsystem.detectedGamePiece()) {
        m_intakeSubsystem.unsqueeze();
      } else {
        // in case we're holding a game piece, we want to keep a hold of it
        m_intakeSubsystem.squeeze();
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
