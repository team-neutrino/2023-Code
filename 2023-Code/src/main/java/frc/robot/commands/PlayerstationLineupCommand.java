// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.SubsystemContainer;
import frc.robot.TrajectoryConfigConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.IntakeManager;
import frc.robot.util.ViennaPIDController;
import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlayerstationLineupCommand extends ParallelCommandGroup {
  private SubsystemContainer m_subsystemContainer;
  private DriveTrainSubsystem m_drivetrainSubsystem;
  private LimelightSubsystem m_limelightSubsystem;

  private Trajectory trajectory;

  /** Creates a new PlayerstationLineupCommand. */
  public PlayerstationLineupCommand(
      SubsystemContainer p_subsystemContainer,
      IntakeManager p_intakeManager,
      ViennaPIDController p_pidController,
      XboxController p_controller) {
    m_subsystemContainer = p_subsystemContainer;
    m_drivetrainSubsystem = m_subsystemContainer.getDriveTrainSubsystem();
    m_limelightSubsystem = m_subsystemContainer.getLimelightSubsystem();
    m_subsystemContainer = p_subsystemContainer;

    double initialDistanceToAprilTag = m_limelightSubsystem.getTargetPoseZ(); // in meters
    Pose2d initialPose = m_drivetrainSubsystem.getPose2d();
    Pose2d finalPose =
        new Pose2d(
            initialPose.getX() + initialDistanceToAprilTag,
            initialPose.getY(),
            initialPose.getRotation());

    Pose2d finalTestPose =
        new Pose2d(initialPose.getX() + 2, initialPose.getY(), initialPose.getRotation());

    trajectory =
        TrajectoryGenerator.generateTrajectory(
            List.of(initialPose, finalPose), TrajectoryConfigConstants.K_MAX_SPEED_FORWARD_CONFIG);

    Trajectory testTrajectory =
        TrajectoryGenerator.generateTrajectory(
            List.of(initialPose, finalTestPose),
            TrajectoryConfigConstants.K_MAX_SPEED_BACKWARD_CONFIG);

    RamseteCommand testRamsete =
        AutonomousUtil.generateRamseteCommand(testTrajectory, m_drivetrainSubsystem);

    // if (m_drivetrainSubsystem.isAngleAcceptable()) {
    //   addCommands(
    //     AutonomousUtil.generateRamseteCommand(
    //       trajectory,
    //       m_drivetrainSubsystem),
    //     new ArmFeederCommand(p_subsystemContainer, p_pidController));
    addCommands(testRamsete, new InstantCommand(() -> m_drivetrainSubsystem.setVoltage(0, 0)));
  }

  private RamseteCommand moveForward() {
    return AutonomousUtil.generateRamseteCommand(trajectory, m_drivetrainSubsystem);
  }
}
