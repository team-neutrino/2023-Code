// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.util.EnumConstants.LEDColor;
import frc.robot.util.ViennaPIDController;

public class ArmToScoreCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private TelescopeSubsystem m_telescopeSubsystem;
  private LEDSubsystem m_ledSubsystem;
  private LimelightSubsystem m_limelightSubsystem;
  private ViennaPIDController m_pidController;

  private double targetArmAngle;
  private double voltage;

  private boolean m_scoreHigh = false;
  private boolean m_scoreBack = true;

  public ArmToScoreCommand(
      SubsystemContainer p_subsystemContainer,
      ViennaPIDController p_pidController,
      XboxController p_drivController,
      boolean p_scoreHigh) {
    m_armSubsystem = p_subsystemContainer.getArmSubsystem();
    m_telescopeSubsystem = p_subsystemContainer.getTelescopeSubsystem();
    m_limelightSubsystem = p_subsystemContainer.getLimelightSubsystem();
    m_ledSubsystem = p_subsystemContainer.getLedSubsystem();
    m_pidController = p_pidController;
    m_scoreHigh = p_scoreHigh;
    addRequirements(m_armSubsystem, m_ledSubsystem, m_telescopeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    /* There is no valid target on the back  */
    if (m_limelightSubsystem.getDistance() < LimelightConstants.SCORING_DISTANCE) {
      m_scoreBack = true;
    } else {
      m_scoreBack = false;
    }

    /* score high from the back */
    if (m_scoreHigh && m_scoreBack && m_ledSubsystem.getColor() == LEDColor.PURPLE) {
      targetArmAngle = ArmConstants.BACK_HIGH_CUBE;
    } else if (m_scoreHigh && m_scoreBack && m_ledSubsystem.getColor() == LEDColor.YELLOW) {
      targetArmAngle = ArmConstants.BACK_HIGH_CONE;
    }

    /* score mid from the back */
    else if (!m_scoreHigh && m_scoreBack && m_ledSubsystem.getColor() == LEDColor.PURPLE) {
      targetArmAngle = ArmConstants.BACK_MID_CUBE;
    } else if (!m_scoreHigh && m_scoreBack && m_ledSubsystem.getColor() == LEDColor.YELLOW) {
      targetArmAngle = ArmConstants.BACK_MID_CONE;
    }

    /* score mid from the front */
    else if (!m_scoreHigh && !m_scoreBack && m_ledSubsystem.getColor() == LEDColor.PURPLE) {
      targetArmAngle = ArmConstants.FRONT_MID_CUBE;
    } else if (!m_scoreHigh && !m_scoreBack && m_ledSubsystem.getColor() == LEDColor.YELLOW) {
      targetArmAngle = ArmConstants.FRONT_MID_CONE;
    }

    voltage = m_pidController.run(m_armSubsystem.getAbsoluteArmPosition(), targetArmAngle);
    m_armSubsystem.smartArmSet(voltage);

    if(m_armSubsystem.atArmPosition(targetArmAngle)){

    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
