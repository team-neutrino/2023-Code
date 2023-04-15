// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeDefaultCommand extends CommandBase {

  private TelescopeSubsystem m_telescopeSubsystem;

  public TelescopeDefaultCommand(SubsystemContainer p_subsystemContainer) {
    m_telescopeSubsystem = p_subsystemContainer.getTelescopeSubsystem();
    addRequirements(m_telescopeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_telescopeSubsystem.retractTelescoping();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
