// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.Limiter;

public class TelescopeCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private XboxController m_driverController;

  public TelescopeCommand(ArmSubsystem p_armSubsystem, XboxController p_driverController) {
    m_armSubsystem = p_armSubsystem;
    m_driverController = p_driverController;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    System.out.println("get left: " + m_driverController.getLeftY());
    
    m_armSubsystem.setTelescope(Limiter.deadzone(m_driverController.getLeftY(),  ArmConstants.ARM_INPUT_DEADZONE));
  }

  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.turnTelescopeOff();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
