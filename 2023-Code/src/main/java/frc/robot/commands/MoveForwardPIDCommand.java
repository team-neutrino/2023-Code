// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.ViennaPIDController;

public class MoveForwardPIDCommand extends CommandBase {
  private DriveTrainSubsystem m_driveTrainSubsystem;
  private ViennaPIDController m_movePIDController;
  private double baseSpeed = 0.3;
  /** Creates a new MoveForwardPIDCommand. */
  public MoveForwardPIDCommand(
      SubsystemContainer p_subsystemContainer, ViennaPIDController p_movePIDController) {
    m_driveTrainSubsystem = p_subsystemContainer.getDriveTrainSubsystem();
    m_movePIDController = p_movePIDController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = 0;
    double currentYaw = m_driveTrainSubsystem.getYaw();
    currentYaw = Math.round(currentYaw);

    output = m_movePIDController.run(currentYaw, 0);
    m_driveTrainSubsystem.setMotors(baseSpeed - output, baseSpeed + output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
