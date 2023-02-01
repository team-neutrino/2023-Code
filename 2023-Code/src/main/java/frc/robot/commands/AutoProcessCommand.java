package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringSubsystem;

public class AutoProcessCommand extends CommandBase{
  IntakeSubsystem m_intakeSubsystem;
  ArmSubsystem m_armSubsystem;
  ScoringSubsystem m_scoringSubsystem;

  public AutoProcessCommand() {
   addRequirements(m_intakeSubsystem, m_armSubsystem, m_scoringSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intakeSubsystem.unsqueeze();
    m_intakeSubsystem.setIntakeDown();
    m_intakeSubsystem.runIntake();
    if(m_intakeSubsystem.isGamePiece()) {
        m_intakeSubsystem.squeeze();
        m_intakeSubsystem.stopIntake();
        m_scoringSubsystem.setScoringOpen();
        m_armSubsystem.setReference(Constants.ArmConstants.ARM_DOWN);
        if(m_scoringSubsystem.getBeamBreak()) {
            m_intakeSubsystem.unsqueeze();
            m_scoringSubsystem.setScoringClose();
            m_armSubsystem.setReference(Constants.ArmConstants.ARM_DOWN);
        }
    }
    else{}
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
