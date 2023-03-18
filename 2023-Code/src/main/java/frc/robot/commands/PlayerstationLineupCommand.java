// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants;
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
  private DriveTrainSubsystem m_drivetrainSubsystem;
  private LimelightSubsystem m_limelightSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private IntakeManager m_intakeManager;
  private ArmSubsystem m_armSubsystem;
  private ViennaPIDController m_pidController;
  /** Creates a new PlayerstationLineupCommand. */
  public PlayerstationLineupCommand(
      DriveTrainSubsystem p_drivetrainSubsystem,
      LimelightSubsystem p_limelightSubsystem,
      ScoringSubsystem p_scoringSubsystem,
      IntakeManager p_intakeManager,
      ArmSubsystem p_armSubsystem,
      ViennaPIDController p_pidController) {
    m_drivetrainSubsystem = p_drivetrainSubsystem;
    m_limelightSubsystem = p_limelightSubsystem;
    m_scoringSubsystem = p_scoringSubsystem;
    m_intakeManager = p_intakeManager;
    m_armSubsystem = p_armSubsystem;
    m_pidController = p_pidController;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // ON START OF COMMAND CALL
    // double initialTx = m_limelightSubsystem.getTx();
    double initialDistance = m_limelightSubsystem.getDistance(); // in meters

    if (m_drivetrainSubsystem.isAngleAcceptable()) {
      addCommands(
          new ScoringOpenCommand(m_scoringSubsystem, m_intakeManager),
          moveForward(initialDistance - DrivetrainConstants.ARM_LENGTH),
          new ArmToAngleCommand(
              m_armSubsystem, m_pidController, m_scoringSubsystem, ArmConstants.PLAYERSTATION));
    }
  }

  private RamseteCommand moveForward(double desiredDistance) {
    ArrayList<PoseTriplet> moveForwardTrajectoryArray =
        new ArrayList<PoseTriplet>(
            Arrays.asList(new PoseTriplet(0, 0, 0), new PoseTriplet(0, desiredDistance, 0)));
    RamseteCommand moveForwardCommand =
        AutonomousUtil.generateRamseteFromPoses(
            moveForwardTrajectoryArray,
            m_drivetrainSubsystem,
            TrajectoryConfigConstants.K_MAX_SPEED_FORWARD_CONFIG);
    return moveForwardCommand;
  }
}
