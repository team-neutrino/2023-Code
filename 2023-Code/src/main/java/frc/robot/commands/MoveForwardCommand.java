// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class MoveForwardCommand extends CommandBase {
  private DriveTrainSubsystem m_drivetrainSubsystem;
  private double m_distance;
  private double m_motorPower;

  public MoveForwardCommand(
      SubsystemContainer p_subsystemContainer, double p_distance, double p_motorPower) {
    m_drivetrainSubsystem = p_subsystemContainer.getDriveTrainSubsystem();
    m_distance = p_distance;
    m_motorPower = p_motorPower;
    addRequirements(m_drivetrainSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_drivetrainSubsystem.setMotors(m_motorPower, m_motorPower);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (m_drivetrainSubsystem.getL1Pos() >= m_distance) {
      return true;
    }
    return false;
  }
}
