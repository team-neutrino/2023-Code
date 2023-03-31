// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.util.ViennaPIDController;

public class ArmAdjustCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private XboxController m_driverController;
  private ViennaPIDController m_pidController;
  private TelescopeSubsystem m_telescopeSubsystem;
  private double targetAngle;

  public ArmAdjustCommand(
      SubsystemContainer p_subsystemContainer,
      XboxController p_driverController,
      ViennaPIDController p_pidController) {
    m_pidController = p_pidController;
    m_armSubsystem = p_subsystemContainer.getArmSubsystem();
    m_driverController = p_driverController;
    m_telescopeSubsystem = p_subsystemContainer.getTelescopeSubsystem();
    targetAngle = m_armSubsystem.getAbsoluteArmPosition();

    addRequirements(m_armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double voltage = 0;

    if (Math.abs(m_driverController.getRightX()) > ArmConstants.ARM_INPUT_DEADZONE) {
      voltage =
          m_armSubsystem.limitArmAmount(
              -m_driverController.getRightX() / ArmConstants.SCALE_FACTOR);
      targetAngle = m_armSubsystem.getAbsoluteArmPosition();
    } else {
      voltage =
          m_pidController.armRun(
              m_armSubsystem.getAbsoluteArmPosition(),
              targetAngle,
              m_telescopeSubsystem.getTelescopingExtension());
    }
    m_armSubsystem.smartArmSet(voltage);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
