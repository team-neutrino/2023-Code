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
    /* if arm is front and past the forward great divide OR if arm is back and is past the back great divide
     * set telescoping out to the max (restricted inside the setTelescope) */

    // if ((m_armSubsystem.getArmVoltage() > 0
    //         && m_armSubsystem.getAbsoluteArmPosition() > ArmConstants.FORWARD_ARM_HEIGHT_LIMIT)
    //     || (m_armSubsystem.getArmVoltage() < 0
    //         && m_armSubsystem.getAbsoluteArmPosition() < ArmConstants.BACKWARD_ARM_HEIGHT_LIMIT)) {
    //   m_telescopeSubsystem.setTelescope(.3);
    // }
    m_telescopeSubsystem.extendTelescoping();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if ((m_armSubsystem.getAbsoluteArmPosition() < ArmConstants.FORWARD_ARM_HEIGHT_LIMIT &&
        m_armSubsystem.getAbsoluteArmPosition() > ArmConstants.BACKWARD_ARM_HEIGHT_LIMIT) ||
        m_armSubsystem.getAbsoluteArmPosition() > 84) {
      System.out.println("end!");
      return true;
    }
    return false;
  }
}
