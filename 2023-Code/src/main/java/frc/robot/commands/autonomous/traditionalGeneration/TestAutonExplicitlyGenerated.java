// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.traditionalGeneration;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryConfigConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAutonExplicitlyGenerated extends SequentialCommandGroup {
  private DriveTrainSubsystem m_drivetrainSubsystem;

  private Trajectory testTrajectory;

  private RamseteCommand testRamseteCommand;

  /** Creates a new TestAutonExplicitlyGenerated. */
  public TestAutonExplicitlyGenerated(DriveTrainSubsystem p_drivetrainSubsystem) {
    m_drivetrainSubsystem = p_drivetrainSubsystem;

    testTrajectory = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(25, 0, Rotation2d.fromDegrees(0))
      ),
      TrajectoryConfigConstants.K_MAX_SPEED_FORWARD_CONFIG
    );

    testRamseteCommand = new RamseteCommand(
      testTrajectory,
      m_drivetrainSubsystem::getPose2d,
      new RamseteController(
          TrajectoryConfigConstants.K_RAMSETE_BETA, TrajectoryConfigConstants.K_RAMSETE_ZETA),
      new SimpleMotorFeedforward(
          TrajectoryConfigConstants.KS_VOLTS,
          TrajectoryConfigConstants.KV_VOLT_SECONDS_PER_METER),
      TrajectoryConfigConstants.K_DRIVE_KINEMATICS,
      m_drivetrainSubsystem::getDriveWheelSpeeds,
      new PIDController(TrajectoryConfigConstants.KP_DRIVE_VEL, 0, 0),
      new PIDController(TrajectoryConfigConstants.KP_DRIVE_VEL, 0, 0),
      m_drivetrainSubsystem::setVoltage,
      m_drivetrainSubsystem);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(testRamseteCommand);
  }
}
