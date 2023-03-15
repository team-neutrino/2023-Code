// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class TelescopreToPositionCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private boolean m_isTelescopingOut;

  public TelescopeCommand(ArmSubsystem p_armSubsystem, boolean p_isTelescopingOut) {
    m_armSubsystem = p_armSubsystem;
    addRequirements(m_armSubsystem);

    m_isTelescopingOut = p_isTelescopingOut;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_isTelescopingOut) {
      m_armSubsystem.setTelescope(ArmConstants.TELESCOPING_OUT_SPEED);
    }
    else {
      m_armSubsystem.setTelescope(ArmConstants.TELESCOPING_IN_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (m_armSubsystem.getSwitch()) {
      return true;
    }
    return false;
  }
}
