package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.IntakeManager;

public class HybridNodeOverideCommand extends CommandBase {
  IntakeSubsystem m_intakeSubsystem;
  IntakeManager m_intakeManager;

  public HybridNodeOverideCommand(
      IntakeSubsystem p_intakeSubsystem, IntakeManager p_inIntakeManager) {
    m_intakeSubsystem = p_intakeSubsystem;
    m_intakeManager = p_inIntakeManager;

    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!m_intakeManager.managerApproved()) {
      return;
    }

    if (m_intakeSubsystem.detectedGamePiece()) {
      m_intakeSubsystem.setIntakeUp();
      m_intakeSubsystem.stopIntake();
      m_intakeSubsystem.squeeze();
    } else {
      m_intakeSubsystem.setIntakeDown();
      m_intakeSubsystem.runIntake();
      m_intakeSubsystem.squeeze();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
