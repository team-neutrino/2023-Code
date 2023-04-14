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
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.NavXBalance;
import frc.robot.commands.ScoringOpenCommand;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.IntakeManager;
import frc.robot.util.PoseTriplet;
import frc.robot.util.ViennaPIDController;
import java.util.*;

public class ScoreMobilityThenBalance extends SequentialCommandGroup {

  private ArrayList<PoseTriplet> forwardBackArray;
  private ArrayList<PoseTriplet> reEnterCommunity;
  private RamseteCommand reEnterCommunityCommand;
  private RamseteCommand moveForwardCommand;

  public ScoreMobilityThenBalance(
      SubsystemContainer p_subsystemContainer,
      ViennaPIDController p_armPidController,
      ViennaPIDController p_balancePidController,
      IntakeManager p_intakeManager,
      XboxController p_driverController) {

    forwardBackArray =
        new ArrayList<PoseTriplet>(
            Arrays.asList(new PoseTriplet(0, 0, 0), new PoseTriplet(3.8, 0, 0)));

    reEnterCommunity =
        new ArrayList<PoseTriplet>(
            Arrays.asList(new PoseTriplet(3.8, 0, 0), new PoseTriplet(1.5, 0, 0)));

    moveForwardCommand =
        AutonomousUtil.generateRamseteFromPoses(
            forwardBackArray,
            p_subsystemContainer.getDriveTrainSubsystem(),
            TrajectoryConfigConstants.K_LESS_SPEED_FORWARD_CONFIG);

    reEnterCommunityCommand =
        AutonomousUtil.generateRamseteFromPoses(
            reEnterCommunity,
            p_subsystemContainer.getDriveTrainSubsystem(),
            TrajectoryConfigConstants.K_LESS_SPEED_BACKWARD_CONFIG);

    addCommands(
        new ArmToAngleCommand(
            p_subsystemContainer,
            p_armPidController,
            p_driverController,
            ArmConstants.BACK_MID,
            true,
            false),
        new ScoringOpenCommand(p_subsystemContainer, p_intakeManager).withTimeout(.75),
        new ArmToAngleCommand(
                p_subsystemContainer,
                p_armPidController,
                p_driverController,
                ArmConstants.FORWARD_MID)
            .withTimeout(1),
        moveForwardCommand,
        new NavXBalance(p_subsystemContainer),
        new AutoBalanceCommand(p_subsystemContainer, p_balancePidController));
  }
}
