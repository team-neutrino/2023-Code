// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubsystemContainer;
import frc.robot.TrajectoryConfigConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.IntakeManager;
import frc.robot.util.PoseTriplet;
import frc.robot.util.ViennaPIDController;
import java.util.*;

public class TestSim extends SequentialCommandGroup {
  private RamseteCommand moveCommand;

  private DriveTrainSubsystem m_drivetrainSubsystem;

  public TestSim(
      SubsystemContainer p_subsystemContainer,
      ViennaPIDController p_pidController,
      XboxController p_driverController,
      IntakeManager p_intakeManager) {
    m_drivetrainSubsystem = p_subsystemContainer.getDriveTrainSubsystem();

    ArrayList<PoseTriplet> moveArray =
        new ArrayList<PoseTriplet>(
            Arrays.asList(
                new PoseTriplet(5, 0, 0),
                new PoseTriplet(2.7, -0.10, -15.08, .32),
                new PoseTriplet(4.08, -0.22, -3.12, .32)));

    moveCommand =
        AutonomousUtil.generateRamseteFromPosesSim(
            moveArray,
            m_drivetrainSubsystem,
            TrajectoryConfigConstants.K_LESS_SPEED_FORWARD_CONFIG);
    addCommands(moveCommand);
  }
}
