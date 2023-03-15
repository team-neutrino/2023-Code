// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.ViennaContainer;
import frc.robot.util.ViennaPIDController;

public class ArmFeederCommand extends CommandBase {
  ArmSubsystem m_armSubsystem;
  ScoringSubsystem m_scoringSubsystem;
  ViennaPIDController m_pidController;
  double voltage;
  double hasGamePiece;
  Timer m_timer;
  double time = 1.5;

  public ArmFeederCommand(
      ArmSubsystem p_armSubsystem,
      ScoringSubsystem p_scoringSubsystem) {
    m_armSubsystem = p_armSubsystem;
    m_scoringSubsystem = p_scoringSubsystem;
    m_pidController = ViennaContainer.getArmSetPositionController();
    m_timer = new Timer();
  }

  @Override
  public void initialize() {
    hasGamePiece = 0;
    m_timer.start();
  }

  @Override
  public void execute() {
    voltage = m_pidController.run(m_armSubsystem.getAbsolutePosition(), ArmConstants.FEEDER);
    m_armSubsystem.set(voltage);

    if (m_scoringSubsystem.detectedGamePiece()) {
      hasGamePiece++;
    } else {
      hasGamePiece = 0;
    }

    if (m_scoringSubsystem.detectedGamePiece() && m_timer.get() > time && hasGamePiece > 18) {
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
