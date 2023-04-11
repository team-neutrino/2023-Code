// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class ExtendModeCommand extends CommandBase {

  private ArmSubsystem m_armSubsystem;
  private TelescopeSubsystem m_telescopeSubsystem;

  public ExtendModeCommand(TelescopeSubsystem p_telescopeSubsystem, ArmSubsystem p_armSubsystem) {
    m_telescopeSubsystem = p_telescopeSubsystem;
    m_armSubsystem = p_armSubsystem;

    addRequirements(m_telescopeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_telescopeSubsystem.extendTelescoping(m_armSubsystem.getAbsoluteArmPosition());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {return false;}
}
