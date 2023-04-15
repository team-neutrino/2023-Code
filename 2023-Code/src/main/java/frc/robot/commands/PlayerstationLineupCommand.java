// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.SubsystemContainer;
import frc.robot.TrajectoryConfigConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.IntakeManager;
import frc.robot.util.PoseTriplet;
import frc.robot.util.ViennaPIDController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlayerstationLineupCommand extends ParallelCommandGroup {
  private SubsystemContainer m_subsystemContainer;
  private DriveTrainSubsystem m_drivetrainSubsystem;
  private LimelightSubsystem m_limelightSubsystem;
  private boolean m_isRight;

  private Trajectory trajectory;

  /** Creates a new PlayerstationLineupCommand. */
  public PlayerstationLineupCommand(
      SubsystemContainer p_subsystemContainer,
      IntakeManager p_intakeManager,
      ViennaPIDController p_pidController,
      XboxController p_controller,
      boolean isRight) {
    m_subsystemContainer = p_subsystemContainer;
    m_drivetrainSubsystem = m_subsystemContainer.getDriveTrainSubsystem();
    m_limelightSubsystem = m_subsystemContainer.getLimelightSubsystem();
    m_subsystemContainer = p_subsystemContainer;
    m_isRight = isRight;

    if (true){//Math.abs(m_drivetrainSubsystem.getYaw() - DrivetrainConstants.PLAYERSTATION_ANGLE) < DrivetrainConstants.ANGLE_DEADZONE) {
      addCommands(
        new InstantCommand(() -> this.testInitialize(isRight)), 
        //new InstantCommand(this::finalAngleAdjust),
        new InstantCommand(() -> m_drivetrainSubsystem.setVoltage(0, 0)));
    }
  }

  private void testInitialize(boolean isRight) {
    Pose2d initialPose = m_drivetrainSubsystem.getPose2d();
    Rotation2d initialRotation = initialPose.getRotation();
    double initialDistanceToAprilTag = m_limelightSubsystem.getTargetPoseZ();
    double initialHorizontalOffset = m_limelightSubsystem.getTargetPoseX();
    double limelightMeasuredAngle = m_limelightSubsystem.getTx();
    Rotation2d adjustRotation = Rotation2d.fromDegrees(initialRotation.getDegrees() - limelightMeasuredAngle);
    double offset;
    if(isRight) {
      offset = 1;
    }
    else {
      offset = -0.6;
    }

    Pose2d finalTestPose =
        new Pose2d(
          initialPose.getX() + initialDistanceToAprilTag*initialRotation.getCos(), 
          initialPose.getY() + initialDistanceToAprilTag*initialRotation.getSin() + initialHorizontalOffset*.5 + offset, 
          Rotation2d.fromDegrees(0));
    System.out.println("initialPose: " + initialPose);
    System.out.println("finalPose: " + finalTestPose);

    Trajectory testTrajectory =
      TrajectoryGenerator.generateTrajectory(
          List.of(initialPose, finalTestPose),
          TrajectoryConfigConstants.K_LESS_SPEED_BACKWARD_CONFIG);

    RamseteCommand testRamsete =
      AutonomousUtil.generateRamseteCommand(testTrajectory, m_drivetrainSubsystem);
    CommandScheduler.getInstance().schedule(testRamsete);
  }

  private void finalAngleAdjust() {
    Pose2d initialPose = m_drivetrainSubsystem.getPose2d();
    Rotation2d initialRotation = initialPose.getRotation();
    double limelightMeasuredAngle = m_limelightSubsystem.getTx();
    double adjustAngle = initialRotation.getDegrees() + limelightMeasuredAngle;
    System.out.println("adjustAngle: " + adjustAngle);

    Pose2d finalTestPose =
        new Pose2d(
          initialPose.getX() + 0.05, 
          initialPose.getY() + 0.05, 
          Rotation2d.fromDegrees(adjustAngle + initialRotation.getDegrees()));
    System.out.println("initialPose: " + initialPose);
    System.out.println("finalPose: " + finalTestPose);

    Trajectory testTrajectory =
      TrajectoryGenerator.generateTrajectory(
          List.of(initialPose, finalTestPose),
          TrajectoryConfigConstants.K_LESS_SPEED_FORWARD_CONFIG);

    RamseteCommand testRamsete =
      AutonomousUtil.generateRamseteCommand(testTrajectory, m_drivetrainSubsystem);
    CommandScheduler.getInstance().schedule(testRamsete);
  }
}
