package frc.robot.commands.autonomous.manualGeneration;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.TrajectoryConfigConstants;
import frc.robot.commands.ArmGatherModeCommand;
import frc.robot.commands.ArmToAngleCommand;
import frc.robot.commands.PrintCommand;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Test1 extends SequentialCommandGroup {
    private DriveTrainSubsystem m_drivetrainSubsystem;
  private ArrayList<PoseTriplet> toGamePieceArray;
  private ArrayList<PoseTriplet> leaveCommunity;
  private ArrayList<PoseTriplet> runThatBack;
  private RamseteCommand toGamePieceCommand;
  private RamseteCommand runThatBackCommand;

  /** Creates a new TestAutonGeneratedTrajectory. */
  public Test1(
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
        new PrintCommand());
  }
}
