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

  public TelescopeCommand(ArmSubsystem p_armSubsystem, boolean p_isExtending) {
    m_armSubsystem = p_armSubsystem;
    addRequirements(m_armSubsystem);

    m_isExtending = p_isExtending;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // System.out.println(m_armSubsystem.getSwitch());
    if (m_isExtending) {
      m_armSubsystem.setTelescope(ArmConstants.TELESCOPE_EXTEND_SPEED);
    } else {
      m_armSubsystem.setTelescope(ArmConstants.TELESCOPE_RETRACT_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (m_armSubsystem.getSwitch() && !m_isExtending) {
      return true;
    }
    return false;
  }
}
