// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubsystemContainer;
import frc.robot.TrajectoryConfigConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
import frc.robot.util.AutonomousUtil;
import frc.robot.util.IntakeManager;
import frc.robot.util.PoseTriplet;
import frc.robot.util.ViennaPIDController;
import java.util.ArrayList;
import java.util.Arrays;

public class JustScore extends SequentialCommandGroup {

  private ArrayList<PoseTriplet> moveBackArray;
  private ArrayList<PoseTriplet> forwardBackArray;

  private RamseteCommand moveBackCommand;
  private RamseteCommand forwardBackCommand;

  private DriveTrainSubsystem m_drivetrainSubsystem;

  /** Creates a new TestAutonGeneratedTrajectory. */
  public JustScore(
      SubsystemContainer p_subsystemContainer,
      ViennaPIDController p_pidController,
      IntakeManager p_intakeManager) {
    m_drivetrainSubsystem = p_subsystemContainer.getDriveTrainSubsystem();

    forwardBackArray =
        new ArrayList<PoseTriplet>(
            Arrays.asList(new PoseTriplet(0, 0, 0), new PoseTriplet(2, 0, 0)));

    forwardBackCommand =
        AutonomousUtil.generateRamseteFromPoses(
            forwardBackArray,
            m_drivetrainSubsystem,
            TrajectoryConfigConstants.K_MAX_SPEED_FORWARD_CONFIG);

    moveBackArray =
        new ArrayList<PoseTriplet>(
            Arrays.asList(new PoseTriplet(2, 0, 0), new PoseTriplet(0, 0, 0)));

    moveBackCommand =
        AutonomousUtil.generateRamseteFromPoses(
            moveBackArray,
            m_drivetrainSubsystem,
            TrajectoryConfigConstants.K_LESS_SPEED_BACKWARD_CONFIG);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        forwardBackCommand, moveBackCommand
        // new ArmToAngleCommand(
        //     p_armSubsystem, p_pidController, ArmConstants.BACK_MID, true, false, p_ledSubsystem),
        // new ScoringOpenCommand(p_scoringSubsystem, p_intakeSubsystem, p_intakeManager, 2, true)
        );
  }
}
