// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.SubsystemContainer;
import frc.robot.TrajectoryConfigConstants;
import frc.robot.commands.ArmToAngleCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.NavXBalance;
import frc.robot.commands.ScoringOpenCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.IntakeManager;
import frc.robot.util.PoseTriplet;
import frc.robot.util.ViennaPIDController;
import java.util.ArrayList;
import java.util.Arrays;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreMobilityThenBalance extends SequentialCommandGroup {

  private ArrayList<PoseTriplet> forwardBackArray;
  private ArrayList<PoseTriplet> reEnterCommunity;
  private RamseteCommand reEnterCommunityCommand;
  private RamseteCommand moveForwardCommand;

  private DriveTrainSubsystem m_drivetrainSubsystem;

  /** Creates a new TestAutonGeneratedTrajectory. */
  public ScoreMobilityThenBalance(
      SubsystemContainer p_subsystemContainer,
      ViennaPIDController p_pidController,
      IntakeManager p_intakeManager) {
    m_drivetrainSubsystem = p_subsystemContainer.getDriveTrainSubsystem();

    forwardBackArray =
        new ArrayList<PoseTriplet>(
            Arrays.asList(new PoseTriplet(0, 0, 0), new PoseTriplet(3.8, 0, 0)));

    reEnterCommunity =
        new ArrayList<PoseTriplet>(
            Arrays.asList(new PoseTriplet(3.8, 0, 0), new PoseTriplet(1.5, 0, 0)));

    moveForwardCommand =
        AutonomousUtil.generateRamseteFromPoses(
            forwardBackArray,
            m_drivetrainSubsystem,
            TrajectoryConfigConstants.K_LESS_SPEED_FORWARD_CONFIG);

    reEnterCommunityCommand =
        AutonomousUtil.generateRamseteFromPoses(
            reEnterCommunity,
            m_drivetrainSubsystem,
            TrajectoryConfigConstants.K_LESS_SPEED_BACKWARD_CONFIG);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ArmToAngleCommand(
            p_subsystemContainer, p_pidController, ArmConstants.BACK_MID, true, false),
        new ScoringOpenCommand(p_subsystemContainer, p_intakeManager),
        new ParallelRaceGroup(
            new TimerCommand(1),
            new ArmToAngleCommand(p_subsystemContainer, p_pidController, ArmConstants.FORWARD_MID)),
        moveForwardCommand,
        new NavXBalance(p_subsystemContainer),
        new AutoBalanceCommand(p_subsystemContainer));
  }
}
