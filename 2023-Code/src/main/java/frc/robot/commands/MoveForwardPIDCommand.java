// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.ViennaPIDController;

public class MoveForwardPIDCommand extends CommandBase {
  private DriveTrainSubsystem m_drivetrainSubsystem;
  private ViennaPIDController m_movePIDController;
  private double baseSpeed = 0.3;

  public MoveForwardPIDCommand(
      SubsystemContainer p_subsystemContainer, ViennaPIDController p_movePIDController) {
    m_drivetrainSubsystem = p_subsystemContainer.getDriveTrainSubsystem();
    m_movePIDController = p_movePIDController;
    addRequirements(m_drivetrainSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double output = 0;
    double currentYaw = m_drivetrainSubsystem.getYaw();
    currentYaw = Math.round(currentYaw);

    output = m_movePIDController.run(currentYaw, 0);
    m_drivetrainSubsystem.setMotors(baseSpeed - output, baseSpeed + output);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
