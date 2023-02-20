// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmToAngleCommand;
import frc.robot.commands.ScoringCloseCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.PoseTriplet;
import frc.robot.util.ViennaPIDController;
import java.util.ArrayList;
import java.util.Arrays;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmBackCommand extends SequentialCommandGroup {
  /** Creates a new ArmBackCommand. */
  ArmSubsystem m_ArmSubsystem;

  DriveTrainSubsystem m_DriveTrainSubsystem;
  ViennaPIDController m_ViennaPIDController;
  ScoringSubsystem m_ScoringSubsystem;

  public ArmBackCommand(
      ArmSubsystem p_ArmSubsystem,
      DriveTrainSubsystem p_DriveTrainSubsystem,
      ViennaPIDController p_ViennaController,
      ScoringSubsystem p_ScoringSubsystem) {
    m_ArmSubsystem = p_ArmSubsystem;
    m_DriveTrainSubsystem = p_DriveTrainSubsystem;
    m_ViennaPIDController = p_ViennaController;
    m_ScoringSubsystem = p_ScoringSubsystem;

    ArrayList<PoseTriplet> forwardMovementArray =
        new ArrayList<PoseTriplet>(
            Arrays.asList(new PoseTriplet(0, 0, 0), new PoseTriplet(2, 0, 0)));

    RamseteCommand forwardMovementCommand =
        AutonomousUtil.generateRamseteFromPoses(forwardMovementArray, m_DriveTrainSubsystem);

    addCommands(
        new ParallelRaceGroup(
            new ArmToAngleCommand(
                m_ArmSubsystem, m_ViennaPIDController, Constants.ArmConstants.BACK_MID),
            new ScoringCloseCommand(m_ScoringSubsystem)),
        new ParallelCommandGroup(
            forwardMovementCommand,
            new ArmToAngleCommand(
                m_ArmSubsystem, m_ViennaPIDController, Constants.ArmConstants.FORWARD_MID)));
  }
}
