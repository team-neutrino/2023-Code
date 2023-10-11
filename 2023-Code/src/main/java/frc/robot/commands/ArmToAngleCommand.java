// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.util.EnumConstants.LEDColor;
import frc.robot.util.ViennaPIDController;

public class ArmToAngleCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private ViennaPIDController m_pidController;
  private XboxController m_driverController;
  private TelescopeSubsystem m_telescopeSubsystem;
  private double m_targetAngle;
  private double voltage;
  private LEDSubsystem m_ledSubsystem;
  private boolean m_auton = false;

  private boolean m_backCheck = false;
  private boolean started = false;
  private double armAngle;

  public ArmToAngleCommand(
      SubsystemContainer p_subsystemContainer,
      ViennaPIDController p_pidController,
      XboxController p_driverController,
      double p_targetAngle) {
    m_armSubsystem = p_subsystemContainer.getArmSubsystem();
    m_telescopeSubsystem = p_subsystemContainer.getTelescopeSubsystem();
    m_ledSubsystem = p_subsystemContainer.getLedSubsystem();
    m_pidController = p_pidController;
    m_driverController = p_driverController;
    m_targetAngle = p_targetAngle;
    addRequirements(m_armSubsystem);
  }

  public ArmToAngleCommand(
      SubsystemContainer p_subsystemContainer,
      ViennaPIDController p_pidController,
      XboxController p_drivController,
      double p_targetAngle,
      boolean p_auton,
      boolean p_backCheck) {
    m_armSubsystem = p_subsystemContainer.getArmSubsystem();
    m_telescopeSubsystem = p_subsystemContainer.getTelescopeSubsystem();
    m_ledSubsystem = p_subsystemContainer.getLedSubsystem();
    m_pidController = p_pidController;
    m_driverController = p_drivController;
    m_targetAngle = p_targetAngle;
    m_auton = p_auton;
    m_backCheck = p_backCheck;
    addRequirements(m_armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

  
      armAngle = m_targetAngle;

    voltage =
        m_pidController.armRun(
            m_armSubsystem.getAbsoluteArmPosition(),
            armAngle,
            m_telescopeSubsystem.getTelescopingExtension());
    m_armSubsystem.smartArmSet(voltage);
  }

  @Override
  public void end(boolean interrupted) {
    started = false;
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(m_armSubsystem.getAbsoluteArmPosition() - m_targetAngle) < 1 && m_auton) {
      return true;
    }
    return false;
  }
}
