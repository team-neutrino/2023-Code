// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.util.Limiter;

public class TelescopeCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private TelescopeSubsystem m_telescopeSubsystem;
  private XboxController m_driverController;
  private boolean m_isAuton;
  private Timer m_timer;

  public TelescopeCommand(
      SubsystemContainer p_subsystemContainer,
      XboxController p_driverController,
      boolean p_isAuton) {
    m_driverController = p_driverController;
    m_telescopeSubsystem = p_subsystemContainer.getTelescopeSubsystem();
    m_armSubsystem = p_subsystemContainer.getArmSubsystem();
    p_isAuton = m_isAuton;
    addRequirements(m_telescopeSubsystem);
  }

  @Override
  public void initialize() {
    if (m_isAuton) {
      m_timer.start();
    }
  }

  @Override
  public void execute() {
    if (m_isAuton) {
      if (m_telescopeSubsystem.getTelescopingExtension()
          <= TelescopeConstants.TELESCOPING_AUTON_HIGH) {
        m_telescopeSubsystem.setTelescope(
            TelescopeConstants.TELESCOPE_EXTEND_SPEED, m_armSubsystem.getAbsoluteArmPosition());
      } else {
        m_telescopeSubsystem.turnTelescopeOff();
      }
    } else {
      double xboxValue =
          -Limiter.bound(
              (Limiter.deadzone(m_driverController.getLeftY(), ArmConstants.ARM_INPUT_DEADZONE)),
              -.5,
              .5);

      m_telescopeSubsystem.setTelescope(xboxValue, m_armSubsystem.getAbsoluteArmPosition());
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_telescopeSubsystem.turnTelescopeOff();
    m_timer.restart();
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    if (m_timer.get() >= 3) {
      return true;
    }
    return false;
  }
}
