// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.IntakeManager;

public class IntakeCommand extends CommandBase {

  IntakeSubsystem m_intakeSubsystem;
  IntakeManager m_armChecker;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem p_intakeSubsystem, IntakeManager p_armChecker) {
    m_intakeSubsystem = p_intakeSubsystem;
    m_armChecker = p_armChecker;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_armChecker.managerApproved()) {
      m_intakeSubsystem.runIntake();
      m_armChecker.setIntakeDownWithArmCheck();
      m_intakeSubsystem.unsqueeze();
      if (m_intakeSubsystem.isGamePiece()) {
        m_intakeSubsystem.squeeze();
        m_intakeSubsystem.stopIntake();
      }
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
