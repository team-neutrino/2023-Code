// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class ExtendModeCommand extends CommandBase {

  private ArmSubsystem m_armSubsystem;
  private TelescopeSubsystem m_telescopeSubsystem;

  public ExtendModeCommand(SubsystemContainer p_subsystemContainer) {
    m_telescopeSubsystem = p_subsystemContainer.getTelescopeSubsystem();
    m_armSubsystem = p_subsystemContainer.getArmSubsystem();

    addRequirements(m_telescopeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_telescopeSubsystem.extendTelescoping();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if ((m_armSubsystem.getAbsoluteArmPosition() < ArmConstants.FORWARD_ARM_HEIGHT_LIMIT
            && m_armSubsystem.getAbsoluteArmPosition() > ArmConstants.BACKWARD_ARM_HEIGHT_LIMIT)
        || m_armSubsystem.getAbsoluteArmPosition() > 84) {
      return true;
    }
    return false;
  }
}
