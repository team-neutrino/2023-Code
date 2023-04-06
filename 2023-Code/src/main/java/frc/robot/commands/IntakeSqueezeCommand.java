// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSqueezeCommand extends CommandBase {

  IntakeSubsystem m_intakeSubsystem;

  public IntakeSqueezeCommand(SubsystemContainer p_subsystemContainer) {
    m_intakeSubsystem = p_subsystemContainer.getIntakeSubsystem();
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intakeSubsystem.squeeze();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
