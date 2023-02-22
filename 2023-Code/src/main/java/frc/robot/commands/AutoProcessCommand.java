package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringSubsystem;

public class AutoProcessCommand extends CommandBase {
  IntakeSubsystem m_intakeSubsystem;
  ArmSubsystem m_armSubsystem;
  ScoringSubsystem m_scoringSubsystem;

  public AutoProcessCommand(
      IntakeSubsystem p_intakeSubsystem,
      ArmSubsystem p_armSubsystem,
      ScoringSubsystem p_scoringSubsystem) {
    m_intakeSubsystem = p_intakeSubsystem;
    m_armSubsystem = p_armSubsystem;
    m_scoringSubsystem = p_scoringSubsystem;
    addRequirements(m_intakeSubsystem, m_armSubsystem, m_scoringSubsystem);
  }

  @Override
  public void initialize() {}
    

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
