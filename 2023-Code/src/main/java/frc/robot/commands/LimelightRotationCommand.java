// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.ViennaPIDController;

public class LimelightRotationCommand extends CommandBase {

  private LimelightSubsystem m_limelightSubsystem;
  private DriveTrainSubsystem m_drivetrainSubsystem;

  private ViennaPIDController pidController;
  
  private double tx;
  private double currentYaw;

  /** Creates a new LimelightRotationCommand. */
  public LimelightRotationCommand(LimelightSubsystem p_limelightSubsystem, DriveTrainSubsystem p_drivetrainSubsystem) {
    m_limelightSubsystem = p_limelightSubsystem;
    m_drivetrainSubsystem = p_drivetrainSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrainSubsystem);
    pidController = new ViennaPIDController();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tx = m_limelightSubsystem.getTx();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentYaw = m_drivetrainSubsystem.getYaw();
    double output = pidController.run(currentYaw, tx);
    m_drivetrainSubsystem.turn(output);
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
