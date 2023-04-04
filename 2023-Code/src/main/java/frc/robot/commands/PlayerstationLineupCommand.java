// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.SubsystemContainer;
import frc.robot.TrajectoryConfigConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.IntakeManager;
import frc.robot.util.PoseTriplet;
import frc.robot.util.ViennaPIDController;
import java.util.ArrayList;
import java.util.Arrays;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlayerstationLineupCommand extends ParallelCommandGroup {
  private SubsystemContainer m_subsystemContainer;
  private DriveTrainSubsystem m_drivetrainSubsystem;
  private LimelightSubsystem m_limelightSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private IntakeManager m_intakeManager;
  private ArmSubsystem m_armSubsystem;
  private ViennaPIDController m_pidController;
  private XboxController m_controller;
  private PoseTriplet m_initialPoseTriplet;
  private PoseTriplet m_targetPoseTriplet;

  /** Creates a new PlayerstationLineupCommand. */
  public PlayerstationLineupCommand(
      SubsystemContainer p_subsystemContainer,
      IntakeManager p_intakeManager,
      ViennaPIDController p_pidController,
      XboxController p_controller) {
    m_subsystemContainer = p_subsystemContainer;
    m_drivetrainSubsystem = m_subsystemContainer.getDriveTrainSubsystem();
    m_limelightSubsystem = m_subsystemContainer.getLimelightSubsystem();
    m_scoringSubsystem = m_subsystemContainer.getScoringSubsystem();
    m_armSubsystem = m_subsystemContainer.getArmSubsystem();
    m_pidController = p_pidController;
    m_controller = p_controller;
    m_subsystemContainer = p_subsystemContainer;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // ON START OF COMMAND CALL
    // double initialTx = m_limelightSubsystem.getTx();
    double initialDistance = m_limelightSubsystem.getDistance(); // in meters
    Pose2d initialPose = m_drivetrainSubsystem.getPose2d();

    m_initialPoseTriplet = new PoseTriplet(
      initialPose.getX(), 
      initialPose.getY(), 
      m_drivetrainSubsystem.getYaw());

    m_targetPoseTriplet = new PoseTriplet(
      initialPose.getX() + initialDistance - DrivetrainConstants.ARM_LENGTH, 
      initialPose.getY(),
      m_drivetrainSubsystem.getYaw());

    if (m_drivetrainSubsystem.isAngleAcceptable()) {
      addCommands(
          moveForward(),
          new ArmFeederCommand(p_subsystemContainer, p_pidController));
    }
  }

  private RamseteCommand moveForward() {
    ArrayList<PoseTriplet> moveForwardTrajectoryArray =
        new ArrayList<PoseTriplet>(Arrays.asList(m_initialPoseTriplet, m_targetPoseTriplet));
    RamseteCommand moveForwardCommand =
        AutonomousUtil.generateRamseteFromPoses(
            moveForwardTrajectoryArray,
            m_drivetrainSubsystem,
            TrajectoryConfigConstants.K_MAX_SPEED_FORWARD_CONFIG);
    return moveForwardCommand;
  }
}
