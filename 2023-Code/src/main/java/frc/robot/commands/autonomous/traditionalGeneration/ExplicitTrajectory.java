// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.traditionalGeneration;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.TrajectoryConfigConstants;

/** Add your docs here. */
public class ExplicitTrajectory {
    Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(25, 0, Rotation2d.fromDegrees(0))
        ),
        TrajectoryConfigConstants.K_MAX_SPEED_FORWARD_CONFIG
    );
}