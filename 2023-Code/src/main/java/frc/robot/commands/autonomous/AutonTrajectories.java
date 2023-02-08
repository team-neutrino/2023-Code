// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.PoseTriplet;
import java.util.ArrayList;
import java.util.Arrays;

public class AutonTrajectories {
  
  /*Relative to robot bumpers aligned with grid */
  private static final PoseTriplet START = new PoseTriplet(0, 0, 0);
  private static final PoseTriplet CHARGER_PAD = new PoseTriplet(10, 0, 0);
  private static final PoseTriplet GAME_PIECE = new PoseTriplet(20, 0, 0);

  public static final ArrayList<PoseTriplet> GRID_TO_GAMEPIECE =
        new ArrayList<PoseTriplet>(
            Arrays.asList(START, GAME_PIECE));

  public static final ArrayList<PoseTriplet> GRID_TO_CHARGER =
        new ArrayList<PoseTriplet>(
            Arrays.asList(START, CHARGER_PAD));

  public static final ArrayList<PoseTriplet> GAMEPIECE_CHARGER =
        new ArrayList<PoseTriplet>(
            Arrays.asList(START, CHARGER_PAD));

  /*Ramsete Commands*/
  private RamseteCommand GRID_TO_GAMEPIECE_COMMAND =
  AutonomousUtil.generateRamseteFromPoses(gr, m_driveTrainSubsystem, TrajectoryConfigConstants.m_ForwardConfig);


}
