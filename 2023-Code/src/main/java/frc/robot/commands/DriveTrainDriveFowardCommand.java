// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class DriveTrainDriveFowardCommand extends CommandBase {
  /** Creates a new DriveTrainDriveFowardCommand. */
  DriveTrainSubsystem m_drivetrain;
  LimelightSubsystem m_limelight;
  double sideX;
  double sideY;
  double sideZ;
  double theta;
  double alpha;
  double sideF;

  public DriveTrainDriveFowardCommand(
      DriveTrainSubsystem p_drivetrain, LimelightSubsystem p_limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = p_drivetrain;
    m_limelight = p_limelight;
    addRequirements(m_drivetrain, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  //negative == left
  //positive == right
  @Override
  public void execute() {
    if (true) {
        sideX = m_limelight.getDistance();
        sideY = m_limelight.getStraightDistance();
        theta = m_limelight.getTx();
        sideZ = m_limelight.lawOfCosines(sideX, sideY, theta);
        alpha = m_limelight.lawOfSines(sideY, sideZ, theta);
        sideF = Math.cos(alpha) * sideX;
        
      }
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
