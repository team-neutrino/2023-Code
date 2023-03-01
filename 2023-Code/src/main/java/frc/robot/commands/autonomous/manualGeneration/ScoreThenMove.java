// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.manualGeneration;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmToAngleCommand;
import frc.robot.commands.MoveForwardCommand;
import frc.robot.commands.ScoringOpenCommand;
import frc.robot.commands.autonomous.TimerCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.PoseTriplet;
import frc.robot.util.ViennaPIDController;
import java.util.ArrayList;
import java.util.Arrays;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreThenMove extends SequentialCommandGroup {

  private DriveTrainSubsystem m_drivetrainSubsystem;
  private ArrayList<PoseTriplet> forwardBackArray;
  private RamseteCommand forwardBackCommand;

  /** Creates a new TestAutonGeneratedTrajectory. */
  public ScoreThenMove(
      DriveTrainSubsystem p_drivetrainSubsystem,
      ViennaPIDController p_pidController,
      ArmSubsystem p_armSubsystem,
      ScoringSubsystem p_scoringSubsystem) {
    m_drivetrainSubsystem = p_drivetrainSubsystem;

    forwardBackArray =
        new ArrayList<PoseTriplet>(
            Arrays.asList(new PoseTriplet(0, 0, 0), new PoseTriplet(3, 0, 0)));

    forwardBackCommand =
        AutonomousUtil.generateRamseteFromPoses(forwardBackArray, m_drivetrainSubsystem);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ArmToAngleCommand(p_armSubsystem, p_pidController, ArmConstants.BACK_MID, true),
        new SequentialCommandGroup(
          new ParallelRaceGroup(
            new ScoringOpenCommand(p_scoringSubsystem),
            new TimerCommand(2)),
          forwardBackCommand
        )
      );

      //forwardBackCommand
      // new ParallelCommandGroup(
      //   new ArmToAngleCommand(p_armSubsystem, p_pidController, ArmConstants.BACK_MID, true),
      //   forwardBackCommand
      // )
    
    
  }
}
