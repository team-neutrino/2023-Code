// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
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

  private boolean m_buttoncheck = false;

  public ArmToAngleCommand(
      ArmSubsystem p_armSubsystem,
      ViennaPIDController p_pidController,
      XboxController p_driverController, TelescopeSubsystem p_telescopeSubsystem,
      double p_targetAngle) {
    m_armSubsystem = p_armSubsystem;
    m_pidController = p_pidController;
    m_driverController = p_driverController;
    m_telescopeSubsystem = p_telescopeSubsystem;
    m_targetAngle = p_targetAngle;

    addRequirements(m_armSubsystem);
  }

  public ArmToAngleCommand(
      ArmSubsystem p_armSubsystem,
      ViennaPIDController p_pidController,
      XboxController p_drivController, TelescopeSubsystem p_telescopeSubsystem,
      double p_targetAngle,
      boolean p_auton,
      boolean p_buttoncheck,
      LEDSubsystem p_ledSubsystem) {
    m_armSubsystem = p_armSubsystem;
    m_pidController = p_pidController;
    m_driverController = p_drivController;
    m_telescopeSubsystem = p_telescopeSubsystem;
    m_targetAngle = p_targetAngle;
    m_auton = p_auton;
    m_buttoncheck = p_buttoncheck;
    m_ledSubsystem = p_ledSubsystem;
    addRequirements(m_armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // if button is X or B and the LED indicates a cone, move arm angle higher than preset.
    // Otherwise assume cube and usual angle.
    if (m_buttoncheck && m_ledSubsystem.getColor() == LEDColor.YELLOW) {
      voltage =
          m_pidController.run(
              m_armSubsystem.getAbsoluteArmPosition(), m_targetAngle + ArmConstants.CONE_ADDITION);
      m_armSubsystem.smartArmSet(voltage);
    } else {
      voltage = m_pidController.run(m_armSubsystem.getAbsoluteArmPosition(), m_targetAngle);
      m_armSubsystem.smartArmSet(voltage);
    }
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
