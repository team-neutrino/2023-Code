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
  double sideA;
  double sideB;
  double sideC;
  double gamma;
  double alpha;
  double theta;
  double sideL;

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
        sideA = m_limelight.getDistance();
        sideB = m_limelight.getStraightDistance();
        gamma = m_limelight.getTx();
        sideC = m_limelight.lawOfCosines(sideA, sideB, gamma);
        alpha = m_limelight.lawOfSines(sideA, sideB, gamma);
        theta = Math.PI - alpha - gamma;
        sideL = sideA * Math.cos(theta);
        
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
