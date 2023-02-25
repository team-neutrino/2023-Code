// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.traditionalGeneration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.TrajectoryConfigConstants;
import frc.robot.commands.ArmToAngleCommand;
import frc.robot.commands.ScoringOpenCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.ViennaPIDController;
import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreThenDriveForwardAuton extends SequentialCommandGroup {
  private DriveTrainSubsystem m_drivetrainSubsystem;
  private RamseteCommand m_driveForwardRamsete;
  private ArmSubsystem m_armSubsystem;
  private ViennaPIDController m_pidController;
  private ScoringSubsystem m_scoringSubsystem;

  /** Creates a new TestAutonExplicitlyGenerated. */
  public ScoreThenDriveForwardAuton(
      DriveTrainSubsystem p_drivetrainSubsystem,
      ArmSubsystem p_armSubsystem,
      ViennaPIDController p_pidController,
      ScoringSubsystem p_scoringSubsystem) {
    m_drivetrainSubsystem = p_drivetrainSubsystem;
    m_armSubsystem = p_armSubsystem;
    m_pidController = p_pidController;
    m_scoringSubsystem = p_scoringSubsystem;

    // Trajectory driveForwardTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         List.of(
    //             new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    //             new Pose2d(5, 0, Rotation2d.fromDegrees(0))),
    //         TrajectoryConfigConstants.K_MAX_SPEED_FORWARD_CONFIG);

    m_driveForwardRamsete =
        new RamseteCommand(
            Trajectories.forwardTrajectory,
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
    // addCommands(new ArmToAngleCommand(m_armSubsystem, m_pidController, ArmConstants.BACK_MID),
    // new ScoringOpenCommand(m_scoringSubsystem), m_driveForwardRamsete);
    addCommands(
        new ArmToAngleCommand(m_armSubsystem, m_pidController, ArmConstants.BACK_MID),
        new ScoringOpenCommand(m_scoringSubsystem));
    // addCommands(
    //     new ArmToAngleCommand(m_armSubsystem, m_pidController, ArmConstants.BACK_MID),
    //     new ScoringOpenCommand(m_scoringSubsystem),
    //     m_driveForwardRamsete);
  }
}
