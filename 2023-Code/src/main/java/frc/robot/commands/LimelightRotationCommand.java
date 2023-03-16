// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.ViennaPIDController;

public class LimelightRotationCommand extends CommandBase {

  private LimelightSubsystem m_limelightSubsystem;
  private DriveTrainSubsystem m_drivetrainSubsystem;

  private ViennaPIDController pidController;
  
  private double initialTx; // initial limelight tx measurement
  private double initialYaw; // initial NavX yaw measurement
  private double initialAprilAngle; // initial angle of AprilTag, relative to 0 yaw of NavX

  private double currentYaw;
  
  

  /** Creates a new LimelightRotationCommand. */
  public LimelightRotationCommand(LimelightSubsystem p_limelightSubsystem, DriveTrainSubsystem p_drivetrainSubsystem) {
    m_limelightSubsystem = p_limelightSubsystem;
    m_drivetrainSubsystem = p_drivetrainSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrainSubsystem);
    pidController = new ViennaPIDController(
      PIDConstants.ROTATE_P,
      PIDConstants.ROTATE_I,
      PIDConstants.ROTATE_D
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTx = m_limelightSubsystem.getTx(); // relative to current angle of robot
    initialYaw = m_drivetrainSubsystem.getYaw();
    initialAprilAngle = initialYaw - initialTx;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentYaw = m_drivetrainSubsystem.getYaw();
    double output = pidController.run(currentYaw, initialAprilAngle);
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
