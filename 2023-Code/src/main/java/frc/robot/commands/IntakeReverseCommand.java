// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.IntakeManager;

public class IntakeReverseCommand extends CommandBase {

  // An object of the intake subsystem
  private IntakeSubsystem m_intakeSubsystem;
  private IntakeManager m_intakeManager;
  private Timer timer;

  public IntakeReverseCommand(IntakeSubsystem p_intakeSubsystem, IntakeManager p_intakeManager) {
    m_intakeSubsystem = p_intakeSubsystem;
    m_intakeManager = p_intakeManager;
    timer = new Timer();

    addRequirements(p_intakeSubsystem);
  }

  @Override
  public void initialize() {
    timer.start();
    ;
  }

  @Override
  public void execute() {
    if (m_intakeManager.managerApproved()) {
      m_intakeManager.setIntakeDownWithArmCheck();
    }

    if (timer.get() >= 1.7) {
      m_intakeSubsystem.unsqueeze();
      m_intakeSubsystem.runIntakeReverse();
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
