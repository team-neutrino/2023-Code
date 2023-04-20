// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.util.EnumConstants.LEDColor;
import frc.robot.util.ViennaPIDController;

public class ArmMagicCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private ViennaPIDController m_pidController;
  private TelescopeSubsystem m_telescopeSubsystem;
  private double voltage;
  private LEDSubsystem m_ledSubsystem;
  private boolean m_high = false;
  private boolean m_auton = false;

  private double armAngle;

  public ArmMagicCommand(
      SubsystemContainer p_subsystemContainer,
      ViennaPIDController p_pidController,
      boolean p_high,
      boolean p_auton) {
    m_armSubsystem = p_subsystemContainer.getArmSubsystem();
    m_telescopeSubsystem = p_subsystemContainer.getTelescopeSubsystem();
    m_ledSubsystem = p_subsystemContainer.getLedSubsystem();
    m_pidController = p_pidController;
    m_high = p_high;
    m_auton = p_auton;
    addRequirements(m_armSubsystem, m_telescopeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_auton) {
      m_ledSubsystem.setToYellow();
    }
    if (m_ledSubsystem.getColor() == LEDColor.PURPLE) {
      if (m_high) {
        armAngle = ArmConstants.BACK_HIGH_CUBE;
      } else {
        armAngle = ArmConstants.BACK_MID_CUBE;
      }
    } else if (m_ledSubsystem.getColor() == LEDColor.YELLOW) {
      if (m_high) {
        armAngle = ArmConstants.BACK_HIGH_CONE;
      } else {
        armAngle = ArmConstants.BACK_MID_CONE;
      }
    } else {
      armAngle = m_armSubsystem.getAbsoluteArmPosition();
    }

    voltage =
        m_pidController.armRun(
            m_armSubsystem.getAbsoluteArmPosition(),
            armAngle,
            m_telescopeSubsystem.getTelescopingExtension());
    m_armSubsystem.smartArmSet(voltage);
    if (m_high) {
      m_telescopeSubsystem.extendTelescoping(m_armSubsystem.getAbsoluteArmPosition());
    } else {
      m_telescopeSubsystem.retractTelescoping();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
