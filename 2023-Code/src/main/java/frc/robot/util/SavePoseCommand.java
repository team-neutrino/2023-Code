// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SavePoseCommand extends InstantCommand {
  private DriveTrainSubsystem m_driveTrainSubsystem; 
  private String m_filename;
  private FileWriter writer;

  public SavePoseCommand(DriveTrainSubsystem p_driveTrainSubsystem, String p_filename) throws IOException {
    m_driveTrainSubsystem = p_driveTrainSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrainSubsystem);
    m_filename = p_filename;
    writer = new FileWriter(PoseProcessor.absoluteAppend(m_filename), true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      writer.write("test");
    } catch (IOException e) {
      e.printStackTrace();
    }
    // Pose2d currentPose = m_driveTrainSubsystem.getPose2d();
    // double xCoord = currentPose.getX();
    // double yCoord = currentPose.getY();
    // double angle = currentPose.getRotation().getDegrees();
    // String toWrite = xCoord + ", " + yCoord + ", " + angle;

    // try {
    //   writer.write(toWrite + "\n");
    // } catch (IOException e) {
    //   System.out.println("File writing error in SavePoseCommand");
    // }
  }
}
