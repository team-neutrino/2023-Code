// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.manualGeneration;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.PoseTriplet;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAutonGeneratedTrajectory extends SequentialCommandGroup {

  private DriveTrainSubsystem m_drivetrainSubsystem;
  private ArrayList<PoseTriplet> forwardBackArray;
  private RamseteCommand forwardBackCommand;

  /** Creates a new TestAutonGeneratedTrajectory. */
  public TestAutonGeneratedTrajectory(DriveTrainSubsystem p_drivetrainSubsystem) {
    m_drivetrainSubsystem = p_drivetrainSubsystem;

    forwardBackArray = new ArrayList<PoseTriplet>(
    Arrays.asList(new PoseTriplet(0, 0, 0), new PoseTriplet(25, 0, 0))
    );

    forwardBackCommand = AutonomousUtil.generateRamseteFromPoses(forwardBackArray, m_drivetrainSubsystem);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(forwardBackCommand);
  }
}
