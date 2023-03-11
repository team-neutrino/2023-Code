// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.manualGeneration;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants;
import frc.robot.SubsystemContainer;
import frc.robot.TrajectoryConfigConstants;
import frc.robot.commands.ArmGatherModeCommand;
import frc.robot.commands.ArmToAngleCommand;
import frc.robot.commands.IntakeGatherModeCommand;
import frc.robot.commands.ScoringOpenCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.IntakeManager;
import frc.robot.util.PoseTriplet;
import frc.robot.util.ViennaPIDController;
import java.util.ArrayList;
import java.util.Arrays;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedScoreThenMoveThenAutoGather extends SequentialCommandGroup {

  private DriveTrainSubsystem m_drivetrainSubsystem;
  private ArrayList<PoseTriplet> toGamePieceArray;
  private ArrayList<PoseTriplet> runThatBack;
  private RamseteCommand toGamePieceCommand;
  private RamseteCommand runThatBackCommand;

  /** Creates a new TestAutonGeneratedTrajectory. */
  public RedScoreThenMoveThenAutoGather(
      SubsystemContainer p_subsystemContainer,
      DriveTrainSubsystem p_drivetrainSubsystem,
      ViennaPIDController p_pidController,
      ArmSubsystem p_armSubsystem,
      ScoringSubsystem p_scoringSubsystem,
      IntakeSubsystem p_intakeSubsystem,
      IntakeManager p_intakeManager,
      LEDSubsystem p_ledSubsystem) {
    m_drivetrainSubsystem = p_drivetrainSubsystem;

    toGamePieceArray =
        new ArrayList<PoseTriplet>(
            Arrays.asList(
                new PoseTriplet(0, 0, 0),
                new PoseTriplet(2.7, 0.10, 15.08),
                new PoseTriplet(4.08, 0.22, 3.12)));

    runThatBack =
        new ArrayList<PoseTriplet>(
            Arrays.asList(
                new PoseTriplet(4.08, 0.22, 3.12),
                new PoseTriplet(1.3, 0.08, -0.16),
                new PoseTriplet(-.3, .31, .32)));

    // change the degrees and get positions
    // toGamePieceArray =
    //     new ArrayList<PoseTriplet>(
    //         Arrays.asList(new PoseTriplet(0, 0, 0), new PoseTriplet(3.93, 0.13, 2.79)));

    // runThatBack =
    //     new ArrayList<PoseTriplet>(
    //         Arrays.asList(new PoseTriplet(3.93, 0.13, 2.79), new PoseTriplet(-.28, .37, -.87)));

    // toGamePieceArray =
    //     new ArrayList<PoseTriplet>(
    //         Arrays.asList(new PoseTriplet(0, 0, 0), new PoseTriplet(4.2, 0.25, 7.5)));

    // runThatBack =
    //     new ArrayList<PoseTriplet>(
    //         Arrays.asList(new PoseTriplet(4.2, 0.25, 7.5),
    //         new PoseTriplet(2, .35, .39),
    //          new PoseTriplet(.22, .46, .39)));

    toGamePieceCommand =
        AutonomousUtil.generateRamseteFromPoses(
            toGamePieceArray,
            m_drivetrainSubsystem,
            TrajectoryConfigConstants.K_MAX_SPEED_FORWARD_CONFIG);

    runThatBackCommand =
        AutonomousUtil.generateRamseteFromPoses(
            runThatBack,
            p_drivetrainSubsystem,
            TrajectoryConfigConstants.K_MAX_SPEED_BACKWARD_CONFIG);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ArmToAngleCommand(p_subsystemContainer, p_pidController, Constants.ArmConstants.BACK_MID, true, false),
        new ScoringOpenCommand(p_subsystemContainer, p_intakeManager)
            .withTimeout(.5),
        new ParallelRaceGroup(
            new ArmToAngleCommand(p_subsystemContainer, p_pidController, Constants.ArmConstants.FORWARD_MID, false, false),
            toGamePieceCommand,
            new IntakeGatherModeCommand(p_subsystemContainer, p_intakeManager, true)),
        new InstantCommand(p_intakeSubsystem::stopIntake, p_intakeSubsystem),
        new ArmGatherModeCommand(p_subsystemContainer, p_pidController)
            .withTimeout(2),
        runThatBackCommand,
        new ArmToAngleCommand(p_subsystemContainer, p_pidController, Constants.ArmConstants.BACK_MID, true, false),
        new ScoringOpenCommand(p_subsystemContainer, p_scoringSubsystem, p_intakeSubsystem, p_intakeManager, 1, true),
        new ArmToAngleCommand(p_subsystemContainer, p_pidController, Constants.ArmConstants.FORWARD_MID, true, true, false));

    // addCommands(
    //     new ArmToAngleCommand(
    //         p_armSubsystem, p_pidController, ArmConstants.BACK_MID, true, false, p_ledSubsystem),
    //     new ScoringOpenCommand(p_scoringSubsystem, p_intakeSubsystem, p_intakeManager)
    //         .withTimeout(.5),
    //     new ParallelCommandGroup(
    //         toGamePieceCommand,
    //         new AutonArmGatherCommand(
    //             p_armSubsystem, p_scoringSubsystem, p_intakeSubsystem, p_pidController)),
    //     runThatBackCommand,
    //     new ArmToAngleCommand(
    //         p_armSubsystem, p_pidController, ArmConstants.BACK_MID, true, false, p_ledSubsystem),
    //     new ScoringOpenCommand(p_scoringSubsystem, p_intakeSubsystem, p_intakeManager, 2, true));
  }
}
