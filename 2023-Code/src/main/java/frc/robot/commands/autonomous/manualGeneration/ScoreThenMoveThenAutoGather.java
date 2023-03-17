// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.manualGeneration;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.TrajectoryConfigConstants;
import frc.robot.commands.ArmGatherModeCommand;
import frc.robot.commands.ArmToAngleCommand;
import frc.robot.commands.ScoringOpenCommand;
import frc.robot.commands.autonomous.TimerCommand;
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
public class ScoreThenMoveThenAutoGather extends SequentialCommandGroup {

  private DriveTrainSubsystem m_drivetrainSubsystem;
  private ArrayList<PoseTriplet> toGamePieceArray;
  private ArrayList<PoseTriplet> leaveCommunity;
  private ArrayList<PoseTriplet> runThatBack;
  private RamseteCommand toGamePieceCommand;
  private RamseteCommand runThatBackCommand;

  /** Creates a new TestAutonGeneratedTrajectory. */
  public ScoreThenMoveThenAutoGather(
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
            Arrays.asList(new PoseTriplet(0, 0, 0), new PoseTriplet(4, 0, 0)));

    runThatBack =
        new ArrayList<PoseTriplet>(
            Arrays.asList(new PoseTriplet(4, 0, 0), new PoseTriplet(0, 0, 0)));

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
        new ArmToAngleCommand(
            p_armSubsystem, p_pidController, ArmConstants.BACK_MID, true, false, p_ledSubsystem),
        new ParallelRaceGroup(
            new ScoringOpenCommand(p_scoringSubsystem, p_intakeSubsystem, p_intakeManager),
            new TimerCommand(2)),
        toGamePieceCommand,
        new ParallelRaceGroup(
            new ArmGatherModeCommand(
                p_armSubsystem, p_scoringSubsystem, p_intakeSubsystem, p_pidController),
            new TimerCommand(2)),
        runThatBackCommand,
        new ArmToAngleCommand(
            p_armSubsystem, p_pidController, ArmConstants.BACK_MID, true, false, p_ledSubsystem),
        new ScoringOpenCommand(p_scoringSubsystem, p_intakeSubsystem, p_intakeManager, 2, true));
  }
}
