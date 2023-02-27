// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.manualGeneration;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmToAngleCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.ScoringOpenCommand;
import frc.robot.commands.TimerCommand;
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
public class ScoreThenBalance extends SequentialCommandGroup {

  private DriveTrainSubsystem m_drivetrainSubsystem;
  private ViennaPIDController m_pidController;
  private ArmSubsystem m_armSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private ArrayList<PoseTriplet> moveForwardArray;
  private RamseteCommand moveForwardCommand;

  public ScoreThenBalance(
      DriveTrainSubsystem p_drivetrainSubsystem,
      ViennaPIDController p_pidController,
      ArmSubsystem p_armSubsystem,
      ScoringSubsystem p_scoringSubsystem) {
    m_drivetrainSubsystem = p_drivetrainSubsystem;
    m_pidController = p_pidController;
    m_armSubsystem = p_armSubsystem;
    m_scoringSubsystem = p_scoringSubsystem;

    moveForwardArray =
        new ArrayList<PoseTriplet>(
            Arrays.asList(new PoseTriplet(0, 0, 0), new PoseTriplet(1.5, 0, 0)));

    moveForwardCommand =
        AutonomousUtil.generateRamseteFromPoses(moveForwardArray, m_drivetrainSubsystem);

    addCommands(
        new ArmToAngleCommand(m_armSubsystem, m_pidController, ArmConstants.BACK_MID, true),
        new ScoringOpenCommand(m_scoringSubsystem).deadlineWith(new TimerCommand(2)),
        moveForwardCommand,
        new AutoBalanceCommand(m_drivetrainSubsystem));
  }
}
