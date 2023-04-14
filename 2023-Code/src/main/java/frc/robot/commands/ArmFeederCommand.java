// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.util.ViennaPIDController;

public class ArmFeederCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private ViennaPIDController m_pidController;
  private TelescopeSubsystem m_telescopeSubsystem;
  private double voltage;
  private double hasGamePiece;
  private Timer m_timer;
  private double time = 1.5;

  public ArmFeederCommand(
      SubsystemContainer p_subsystemContainer,
      ViennaPIDController p_pidController,
      LEDSubsystem p_ledSubsystem) {
    m_armSubsystem = p_subsystemContainer.getArmSubsystem();
    m_scoringSubsystem = p_subsystemContainer.getScoringSubsystem();
    m_pidController = p_pidController;
    m_telescopeSubsystem = p_subsystemContainer.getTelescopeSubsystem();
    m_timer = new Timer();
    addRequirements(m_armSubsystem);
  }

  @Override
  public void initialize() {
    hasGamePiece = 0;
    m_timer.start();
  }

  @Override
  public void execute() {
    voltage =
        m_pidController.armRun(
            m_armSubsystem.getAbsoluteArmPosition(),
            ArmConstants.FEEDER,
            m_telescopeSubsystem.getTelescopingExtension());
    m_armSubsystem.smartArmSet(voltage);

    if (m_scoringSubsystem.detectedGamePiece()) {
      hasGamePiece++;
    } else {
      hasGamePiece = 0;
    }

    if (m_scoringSubsystem.detectedGamePiece() && m_timer.get() > time && hasGamePiece > 6) {
      m_scoringSubsystem.closeScoring();
    } else {
      m_scoringSubsystem.openScoring();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_timer.reset();
    hasGamePiece = 0;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
