// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.util.ViennaPIDController;

public class ArmToAngleCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private TelescopeSubsystem m_telescopeSubsystem;
  private LEDSubsystem m_ledSubsystem;
  private ViennaPIDController m_pidController;

  private double m_targetAngle;
  private double armAngle;
  private double voltage;

  private boolean m_auton = false;

  public ArmToAngleCommand(
      SubsystemContainer p_subsystemContainer,
      ViennaPIDController p_pidController,
      double p_targetAngle) {
    m_armSubsystem = p_subsystemContainer.getArmSubsystem();
    m_telescopeSubsystem = p_subsystemContainer.getTelescopeSubsystem();
    m_ledSubsystem = p_subsystemContainer.getLedSubsystem();

    m_pidController = p_pidController;
    m_targetAngle = p_targetAngle;

    addRequirements(m_armSubsystem, m_telescopeSubsystem);
  }

  public ArmToAngleCommand(
      SubsystemContainer p_subsystemContainer,
      ViennaPIDController p_pidController,
      double p_targetAngle,
      boolean p_auton) {
    m_armSubsystem = p_subsystemContainer.getArmSubsystem();
    m_telescopeSubsystem = p_subsystemContainer.getTelescopeSubsystem();
    m_ledSubsystem = p_subsystemContainer.getLedSubsystem();

    m_pidController = p_pidController;
    m_targetAngle = p_targetAngle;

    m_auton = p_auton;
    addRequirements(m_armSubsystem, m_ledSubsystem, m_telescopeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    armAngle = m_targetAngle;

    voltage = m_pidController.run(m_armSubsystem.getAbsoluteArmPosition(), armAngle);
    m_armSubsystem.smartArmSet(voltage);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (Math.abs(m_armSubsystem.getAbsoluteArmPosition() - m_targetAngle) < 1 && m_auton) {
      return true;
    }
    return false;
  }
}
