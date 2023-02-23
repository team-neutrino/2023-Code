// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.IntakeManager;

public class IntakeReverseCommand extends CommandBase {

  // An object of the intake subsystem
  private IntakeSubsystem m_intakeSubsystem;
  private IntakeManager m_intakeManager;

  public IntakeReverseCommand(IntakeSubsystem p_intakeSubsystem, IntakeManager p_intakeManager) {
    m_intakeSubsystem = p_intakeSubsystem;
    m_intakeManager = p_intakeManager;

    addRequirements(p_intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // add a delay so intake goes down and then outtakes
    if (m_intakeManager.managerApproved()) {
      m_intakeSubsystem.runIntakeReverse();
      m_intakeManager.setIntakeDownWithArmCheck();
      m_intakeSubsystem.unsqueeze();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
