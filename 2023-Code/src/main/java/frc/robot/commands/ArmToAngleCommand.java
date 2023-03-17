// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.LEDColor;
import frc.robot.util.ViennaPIDController;

public class ArmToAngleCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private ViennaPIDController m_pidController;
  private double m_targetAngle;
  private ScoringSubsystem m_ScoringSubsystem;
  private double voltage;
  private LEDSubsystem m_ledSubsystem;
  private boolean m_auton = false;
  private boolean m_endAuton = false;

  private boolean m_buttoncheck = false;

  public ArmToAngleCommand(
      ArmSubsystem p_armSubsystem,
      ViennaPIDController p_pidController,
      ScoringSubsystem p_ScoringSubsystem,
      double p_targetAngle) {
    m_armSubsystem = p_armSubsystem;
    m_ScoringSubsystem = p_ScoringSubsystem;
    m_pidController = p_pidController;
    m_targetAngle = p_targetAngle;

    addRequirements(m_armSubsystem);
  }

  public ArmToAngleCommand(
      ArmSubsystem p_armSubsystem,
      ViennaPIDController p_pidController,
      double p_targetAngle,
      boolean p_auton,
      boolean p_buttoncheck,
      LEDSubsystem p_ledSubsystem) {
    m_armSubsystem = p_armSubsystem;
    m_pidController = p_pidController;
    m_targetAngle = p_targetAngle;
    m_auton = p_auton;
    m_endAuton = false;
    m_buttoncheck = p_buttoncheck;
    m_ledSubsystem = p_ledSubsystem;
    addRequirements(m_armSubsystem);
  }

  public ArmToAngleCommand(
      ArmSubsystem p_armSubsystem,
      ViennaPIDController p_pidController,
      double p_targetAngle,
      boolean p_auton,
      boolean p_endAuton,
      boolean p_buttoncheck,
      LEDSubsystem p_ledSubsystem) {
    m_armSubsystem = p_armSubsystem;
    m_pidController = p_pidController;
    m_targetAngle = p_targetAngle;
    m_auton = p_auton;
    m_endAuton = p_endAuton;
    m_buttoncheck = p_buttoncheck;
    m_ledSubsystem = p_ledSubsystem;
    addRequirements(m_armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_buttoncheck) {

      if (m_ledSubsystem.getColor() == LEDColor.YELLOW) {
        voltage =
            m_pidController.run(
                m_armSubsystem.getAbsoluteArmPosition(), Constants.ArmConstants.BACK_MID);
        m_armSubsystem.smartArmSet(voltage);
      } else {
        voltage =
            m_pidController.run(
                m_armSubsystem.getAbsoluteArmPosition(), Constants.ArmConstants.QUASI_BACK_MID);
        m_armSubsystem.smartArmSet(voltage);
      }
    } else {
      voltage = m_pidController.run(m_armSubsystem.getAbsoluteArmPosition(), m_targetAngle);
      m_armSubsystem.smartArmSet(voltage);
    }

    if (m_endAuton) {
      m_ScoringSubsystem.openScoring();
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
