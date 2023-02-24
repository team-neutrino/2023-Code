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

public class ArmGatherModeCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private ViennaPIDController m_pidController;

  /** Creates a new ArmGatherModeCommand. */
  public ArmGatherModeCommand(
      ArmSubsystem p_armSubsystem,
      ScoringSubsystem p_scoringSubsystem, IntakeSubsystem p_intakeSubsystem,
      ViennaPIDController p_pidController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = p_armSubsystem;
    m_scoringSubsystem = p_scoringSubsystem;
    m_intakeSubsystem = p_intakeSubsystem;
    m_pidController = p_pidController;

    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_intakeSubsystem.setIntakeDown();
    m_armSubsystem.smartSet(
        m_pidController.run(m_armSubsystem.getAbsolutePosition(), ArmConstants.ARM_FRONTMOST));


    
    if (m_armSubsystem.getAbsolutePosition() >= ArmConstants.GATHER_MODE) {
      if(m_intakeSubsystem.isIntakeDown()){
        m_intakeSubsystem.unsqueeze();
      }
      m_scoringSubsystem.closeScoring();
    } else {
      m_scoringSubsystem.openScoring();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
