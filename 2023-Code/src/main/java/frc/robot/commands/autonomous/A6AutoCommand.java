// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.PoseTriplet;
import java.util.ArrayList;
import java.util.Arrays;

public class A6AutoCommand extends SequentialCommandGroup {
  public A6AutoCommand(DriveTrainSubsystem p_driveTrainSubsystem, ScoringSubsystem p_scoringSubsystem, ArmSubsystem p_armSubsystem, IntakeSubsystem p_intakeSubsystem) {
    ArrayList<PoseTriplet> forwardBackArray =
        new ArrayList<PoseTriplet>(
            Arrays.asList(new PoseTriplet(0, 0, 0), new PoseTriplet(25, 0, 0)));

    RamseteCommand forwardBackCommand =
        AutonomousUtil.generateRamseteFromPoses(forwardBackArray, p_driveTrainSubsystem);

    addCommands(forwardBackCommand);
  }
}
