// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class NavXBalance extends CommandBase {
  DriveTrainSubsystem m_DriveTrainSubsystem;
  double TILTED = -14;

  /** Creates a new NavXBalance. */
  public NavXBalance(SubsystemContainer p_subsystemContainer) {
    m_DriveTrainSubsystem = p_subsystemContainer.getDriveTrainSubsystem();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveTrainSubsystem.setMotors(
        DrivetrainConstants.BALANCE_MOTOR_POWER, DrivetrainConstants.BALANCE_MOTOR_POWER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_DriveTrainSubsystem.getRoll() < TILTED) {
      return true;
    }
    return false;
  }
}
