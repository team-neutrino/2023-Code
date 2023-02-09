// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.PoseProcessor;
import frc.robot.util.PoseTriplet;
import java.util.ArrayList;
import java.util.Arrays;

public class TestAuton extends SequentialCommandGroup {

  public TestAuton(DriveTrainSubsystem p_driveTrainSubsystem) {
    RamseteCommand testCommand =
        AutonomousUtil.generateRamseteFromPoseFile("testFile.txt", p_driveTrainSubsystem);

    addCommands(testCommand);
  }
}
