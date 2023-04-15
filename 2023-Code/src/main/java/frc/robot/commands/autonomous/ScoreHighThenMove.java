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
import frc.robot.commands.TelescopeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.IntakeManager;
import frc.robot.util.PoseTriplet;
import frc.robot.util.ViennaPIDController;
import java.util.*;

public class ScoreHighThenMove extends SequentialCommandGroup {

  private DriveTrainSubsystem m_drivetrainSubsystem;
  private ArmSubsystem m_armSubsystem;
  private ViennaPIDController m_pidController;
  private LEDSubsystem m_ledSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private IntakeManager m_intakeManager;

  private RamseteCommand moveForwardCommand;

  public ScoreHighThenMove(
      SubsystemContainer p_subsystemContainer,
      ViennaPIDController p_pidController,
      IntakeManager p_intakeManager,
      XboxController p_driverController) {
    m_drivetrainSubsystem = p_subsystemContainer.getDriveTrainSubsystem();
    m_armSubsystem = p_subsystemContainer.getArmSubsystem();
    m_pidController = p_pidController;
    m_ledSubsystem = p_subsystemContainer.getLedSubsystem();
    m_scoringSubsystem = p_subsystemContainer.getScoringSubsystem();
    m_intakeManager = p_intakeManager;

    ArrayList<PoseTriplet> moveForwardArray =
        new ArrayList<PoseTriplet>(
            Arrays.asList(new PoseTriplet(0, 0, 0), new PoseTriplet(4, 0, 0)));

    moveForwardCommand =
        AutonomousUtil.generateRamseteFromPoses(
            moveForwardArray,
            m_drivetrainSubsystem,
            TrajectoryConfigConstants.K_MAX_SPEED_FORWARD_CONFIG);

    addCommands(
        new ArmToAngleCommand(
            p_subsystemContainer,
            p_pidController,
            p_driverController,
            ArmConstants.BACK_MID,
            true,
            false),
        new TelescopeCommand(p_subsystemContainer, p_driverController),
        new ScoringOpenCommand(p_subsystemContainer, p_intakeManager).withTimeout(2),
        moveForwardCommand);
  }
}
