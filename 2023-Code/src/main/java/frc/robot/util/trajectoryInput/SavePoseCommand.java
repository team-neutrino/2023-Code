// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.trajectoryInput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SavePoseCommand extends InstantCommand {
  private DriveTrainSubsystem m_driveTrainSubsystem;
  private String m_filename;
  // private BufferedWriter writer;

  public SavePoseCommand(DriveTrainSubsystem p_driveTrainSubsystem, String p_filename)
      throws IOException {
    m_driveTrainSubsystem = p_driveTrainSubsystem;
    m_filename = p_filename;
  }

  private void printPoseToFile(String p_filename, String p_message) {
    File outputFile = new File(PoseProcessor.absoluteAppend("testInput.txt"));
    try {
      BufferedWriter bf = new BufferedWriter(new FileWriter(outputFile, true));
      bf.write(p_message + '\n');
      bf.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentPose = m_driveTrainSubsystem.getPose2d();
    double xCoord = currentPose.getX();
    double yCoord = currentPose.getY();
    double angle = currentPose.getRotation().getDegrees();

    String toWrite = xCoord + ", " + yCoord + ", " + angle;
    printPoseToFile(m_filename, toWrite);
  }
}
