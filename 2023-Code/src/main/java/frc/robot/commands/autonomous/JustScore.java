// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.SubsystemContainer;
import frc.robot.TrajectoryConfigConstants;
import frc.robot.commands.ArmToAngleCommand;
import frc.robot.commands.ScoringOpenCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.IntakeManager;
import frc.robot.util.PoseTriplet;
import frc.robot.util.ViennaPIDController;
import java.util.*;

public class JustScore extends SequentialCommandGroup {

  private ArrayList<PoseTriplet> moveBackArray;
  private ArrayList<PoseTriplet> forwardBackArray;

  private RamseteCommand moveBackCommand;
  private RamseteCommand forwardBackCommand;

  private DriveTrainSubsystem m_drivetrainSubsystem;

  public JustScore(
      SubsystemContainer p_subsystemContainer,
      ViennaPIDController p_pidController,
      XboxController p_driverController,
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
    addCommands(
        forwardBackCommand,
        moveBackCommand,
        new ArmToAngleCommand(
            p_subsystemContainer,
            p_pidController,
            p_driverController,
            ArmConstants.BACK_MID,
            true,
            false),
        new ScoringOpenCommand(p_subsystemContainer, p_intakeManager));
  }
}
