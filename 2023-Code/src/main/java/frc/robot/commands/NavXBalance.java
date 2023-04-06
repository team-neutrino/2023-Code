// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class NavXBalance extends CommandBase {
  DriveTrainSubsystem m_drivetrainSubsystem;
  double TILTED = -14;

  public NavXBalance(SubsystemContainer p_subsystemContainer) {
    m_drivetrainSubsystem = p_subsystemContainer.getDriveTrainSubsystem();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_drivetrainSubsystem.setMotors(
        DrivetrainConstants.BALANCE_MOTOR_POWER, DrivetrainConstants.BALANCE_MOTOR_POWER);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (m_drivetrainSubsystem.getRoll() < TILTED) {
      return true;
    }
    return false;
  }
}
