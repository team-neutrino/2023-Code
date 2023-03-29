// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class TelescopeCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private boolean m_isExtending;

  public TelescopeCommand(SubsystemContainer p_subsystemContainer, boolean p_isExtending) {
    m_armSubsystem = p_subsystemContainer.getArmSubsystem();
    m_isExtending = p_isExtending;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_isExtending) {
      m_armSubsystem.setTelescope(ArmConstants.TELESCOPE_EXTEND_SPEED);
    } else {
      m_armSubsystem.setTelescope(ArmConstants.TELESCOPE_RETRACT_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.turnTelescopeOff();
  }

  @Override
  public boolean isFinished() {
    if (m_armSubsystem.getSwitch() && !m_isExtending) {
      return true;
    }
    return false;
  }
}
