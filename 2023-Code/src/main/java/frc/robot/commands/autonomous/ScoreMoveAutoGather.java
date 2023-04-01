// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.SubsystemContainer;
import frc.robot.TrajectoryConfigConstants;
import frc.robot.commands.ArmGatherModeCommand;
import frc.robot.commands.ArmToAngleCommand;
import frc.robot.commands.IntakeGatherModeCommand;
import frc.robot.commands.ScoringOpenCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.DriverStationInfo;
import frc.robot.util.IntakeManager;
import frc.robot.util.PoseTriplet;
import frc.robot.util.ViennaPIDController;
import java.util.ArrayList;
import java.util.Arrays;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// FOR BLUE PATH
public class ScoreMoveAutoGather extends SequentialCommandGroup {

  private DriveTrainSubsystem m_drivetrainSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private ArrayList<PoseTriplet> toGamePieceArray;
  private ArrayList<PoseTriplet> runThatBack;
  private RamseteCommand toGamePieceCommand;
  private RamseteCommand runThatBackCommand;
  private boolean inverted = false;

  /** Creates a new TestAutonGeneratedTrajectory. */
  public ScoreMoveAutoGather(
      SubsystemContainer p_subsystemContainer,
      ViennaPIDController p_pidController,
      IntakeManager p_intakeManager,
      XboxController p_driverController) {
    m_drivetrainSubsystem = p_subsystemContainer.getDriveTrainSubsystem();
    m_intakeSubsystem = p_subsystemContainer.getIntakeSubsystem();
    if (DriverStationInfo.getAlliance() == Alliance.Red) {
      inverted = true;
    }

    toGamePieceArray =
        new ArrayList<PoseTriplet>(
            Arrays.asList(
                new PoseTriplet(0, 0, 0),
                new PoseTriplet(2.7, -0.10, -15.08),
                new PoseTriplet(4.08, -0.22, -3.12)));

    runThatBack =
        new ArrayList<PoseTriplet>(
            Arrays.asList(
                new PoseTriplet(4.08, -0.22, -3.12),
                new PoseTriplet(1.3, -0.08, 0.16),
                new PoseTriplet(-.3, -.31, -.32)));

    toGamePieceCommand =
        AutonomousUtil.generateRamseteFromPoses(
            toGamePieceArray,
            m_drivetrainSubsystem,
            TrajectoryConfigConstants.K_MAX_SPEED_FORWARD_CONFIG,
            inverted);

    runThatBackCommand =
        AutonomousUtil.generateRamseteFromPoses(
            runThatBack,
            m_drivetrainSubsystem,
            TrajectoryConfigConstants.K_MAX_SPEED_BACKWARD_CONFIG,
            inverted);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ArmToAngleCommand(
            p_subsystemContainer,
            p_pidController,
            p_driverController,
            ArmConstants.BACK_MID,
            true,
            false),
        new ScoringOpenCommand(p_subsystemContainer, p_intakeManager).withTimeout(.5),
        new ParallelRaceGroup(
            new ArmToAngleCommand(
                p_subsystemContainer,
                p_pidController,
                p_driverController,
                ArmConstants.FORWARD_MID,
                false,
                false),
            toGamePieceCommand,
            new IntakeGatherModeCommand(p_subsystemContainer, p_intakeManager, true)),
        new InstantCommand(p_subsystemContainer.getIntakeSubsystem()::stopIntake),
        new ArmGatherModeCommand(p_subsystemContainer, p_pidController).withTimeout(2),
        runThatBackCommand,
        new ArmToAngleCommand(
            p_subsystemContainer,
            p_pidController,
            p_driverController,
            ArmConstants.BACK_MID,
            true,
            false),
        new ScoringOpenCommand(p_subsystemContainer, p_intakeManager).withTimeout(1),
        new ArmToAngleCommand(
                p_subsystemContainer,
                p_pidController,
                p_driverController,
                ArmConstants.FORWARD_MID,
                true,
                false)
            .alongWith(new ScoringOpenCommand(p_subsystemContainer, p_intakeManager)));
  }
}
