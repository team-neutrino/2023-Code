// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class TelescopeCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private boolean m_isExtending;

  private double m_endLength = 0;

  public TelescopeCommand(ArmSubsystem p_armSubsystem, boolean p_isExtending) {
    m_armSubsystem = p_armSubsystem;

    m_isExtending = p_isExtending;
  }

  // arm extends fully if p_endLength is not set
  public TelescopeCommand(ArmSubsystem p_armSubsystem, boolean p_isExtending, double p_endLength) {
    m_armSubsystem = p_armSubsystem;

    m_isExtending = p_isExtending;

    m_endLength = p_endLength;
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
    if (m_endLength != 0 && m_armSubsystem.getAbsoluteArmPosition() <= m_endLength) {
      return true;
    }
    return false;
  }
}
