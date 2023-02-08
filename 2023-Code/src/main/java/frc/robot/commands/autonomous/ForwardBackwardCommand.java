// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryConfigConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.PoseTriplet;
import java.util.ArrayList;
import java.util.Arrays;

public class ForwardBackwardCommand extends SequentialCommandGroup {

  private DriveTrainSubsystem m_driveTrainSubsystem;

  private ArrayList<PoseTriplet> forwardBackArray =
        new ArrayList<PoseTriplet>(
            Arrays.asList(new PoseTriplet(0, 0, 0), new PoseTriplet(25, 0, 0)));

  private RamseteCommand forwardBackCommand =
      AutonomousUtil.generateRamseteFromPoses(forwardBackArray, m_driveTrainSubsystem, TrajectoryConfigConstants.m_ForwardConfig);

  public ForwardBackwardCommand(DriveTrainSubsystem p_driveTrainSubsystem) {
    m_driveTrainSubsystem = p_driveTrainSubsystem;
    addCommands(forwardBackCommand);
  }
}
