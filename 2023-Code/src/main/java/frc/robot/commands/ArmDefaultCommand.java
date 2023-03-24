// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.util.ViennaPIDController;

public class ArmDefaultCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private ViennaPIDController m_pidController;
  private TelescopeSubsystem m_telescopeSubsystem;

  public ArmDefaultCommand(ArmSubsystem p_armSubsystem, TelescopeSubsystem p_telescopeSubsystem, ViennaPIDController p_pidController) {
    m_armSubsystem = p_armSubsystem;
    m_pidController = p_pidController;
    m_telescopeSubsystem = p_telescopeSubsystem;
    addRequirements(m_armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_armSubsystem.smartArmSet(
        m_pidController.armRun(
            m_armSubsystem.getAbsoluteArmPosition(),
            ArmConstants.FORWARD_MID,
            m_telescopeSubsystem.getTelescopingExtension()));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
