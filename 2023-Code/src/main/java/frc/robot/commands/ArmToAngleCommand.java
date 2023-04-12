// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.util.ViennaPIDController;

public class ArmToAngleCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private ViennaPIDController m_pidController;
  private TelescopeSubsystem m_telescopeSubsystem;
  private double m_targetAngle;
  private double voltage;
  private boolean m_auton = false;
  private double armAngle;

  public ArmToAngleCommand(
      SubsystemContainer p_subsystemContainer,
      ViennaPIDController p_pidController,
      XboxController p_driverController,
      double p_targetAngle, boolean p_auton) {
    m_armSubsystem = p_subsystemContainer.getArmSubsystem();
    m_telescopeSubsystem = p_subsystemContainer.getTelescopeSubsystem();
    m_pidController = p_pidController;
    m_targetAngle = p_targetAngle;
    m_auton = p_auton;
    addRequirements(m_armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    voltage =
        m_pidController.armRun(
            m_armSubsystem.getAbsoluteArmPosition(),
            armAngle,
            m_telescopeSubsystem.getTelescopingExtension());
    m_armSubsystem.smartArmSet(voltage);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(m_armSubsystem.getAbsoluteArmPosition() - m_targetAngle) < 1 && m_auton) {
      return true;
    }
    return false;
  }
}
