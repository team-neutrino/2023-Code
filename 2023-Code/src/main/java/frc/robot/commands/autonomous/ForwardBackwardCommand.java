// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.PoseTriplet;
import java.util.ArrayList;
import java.util.Arrays;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ForwardBackwardCommand extends SequentialCommandGroup {

  private DriveTrainSubsystem m_driveTrainSubsystem;

  private final ArrayList<PoseTriplet> forwardBackArray =
      new ArrayList<PoseTriplet>(
          Arrays.asList(
              new PoseTriplet(0, 0, 0), new PoseTriplet(1.5, 0, 0), new PoseTriplet(0, 0, 0)));

  private final RamseteCommand forwardBackCommand =
      AutonomousUtil.generateRamseteFromPoses(forwardBackArray, m_driveTrainSubsystem);

  /** Creates a new ForwardBackwardCommand. */
  public ForwardBackwardCommand(DriveTrainSubsystem p_driveTrainSubsystem) {
    m_driveTrainSubsystem = p_driveTrainSubsystem;
    addCommands(forwardBackCommand);
  }
}
