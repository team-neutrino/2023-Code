// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class TelescopeCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private XboxController m_driverController;
  private boolean m_isExtending;

  public TelescopeCommand(ArmSubsystem p_armSubsystem, XboxController p_driverController,
  boolean p_isExtending) {
    m_armSubsystem = p_armSubsystem;
    m_driverController = p_driverController;
    m_isExtending = p_isExtending;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (Math.abs(m_driverController.getLeftY()) > ArmConstants.ARM_INPUT_DEADZONE) {
      m_armSubsystem.setTelescope(m_driverController.getRightY());
    } else {
      m_armSubsystem.turnTelescopeOff();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.turnTelescopeOff();
  }

  @Override
  public boolean isFinished() {
    if (m_armSubsystem.isPressed() && !m_isExtending) {
      return true;
    }
    return false;
  }
}
