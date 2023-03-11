// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class MoveForwardCommand extends CommandBase {
  private DriveTrainSubsystem m_driveTrainSubsystem;
  private double m_distance;
  private double m_motorPower;

  /** Creates a new MoveForwardCommand. */
  public MoveForwardCommand(
      SubsystemContainer p_subsystemContainer, double p_distance, double p_motorPower) {
    m_driveTrainSubsystem = p_subsystemContainer.getDriveTrainSubsystem();
    m_distance = p_distance;
    m_motorPower = p_motorPower;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrainSubsystem.setMotors(m_motorPower, m_motorPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_driveTrainSubsystem.getL1Pos() >= m_distance) {
      return true;
    }
    return false;
  }
}
