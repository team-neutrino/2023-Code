// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.TrajectoryConfigConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

/** Add your docs here. */
public class AutonomousUtil {

  private final DriveTrainSubsystem m_driveTrainSubsystem = new DriveTrainSubsystem();

  public RamseteCommand GenerateRamseteCommand(Trajectory trajectory) {
    return new RamseteCommand(
        trajectory,
        m_driveTrainSubsystem::getPose2d,
        new RamseteController(
            TrajectoryConfigConstants.K_RAMSETE_BETA, TrajectoryConfigConstants.K_RAMSETE_ZETA),
        new SimpleMotorFeedforward(
            TrajectoryConfigConstants.KS_VOLTS,
            TrajectoryConfigConstants.KV_VOLT_SECONDS_PER_METER),
        TrajectoryConfigConstants.K_DRIVE_KINEMATICS,
        m_driveTrainSubsystem::getDriveWheelSpeeds,
        new PIDController(TrajectoryConfigConstants.KP_DRIVE_VEL, 0, 0),
        new PIDController(TrajectoryConfigConstants.KP_DRIVE_VEL, 0, 0),
        m_driveTrainSubsystem::setVoltage,
        m_driveTrainSubsystem);
  }
}
