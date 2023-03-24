// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.TrajectoryConfigConstants;
import frc.robot.commands.ArmToAngleCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.ScoringOpenCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.IntakeManager;
import frc.robot.util.PoseTriplet;
import frc.robot.util.ViennaPIDController;
import java.util.ArrayList;
import java.util.Arrays;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreThenBalance extends SequentialCommandGroup {

  private ArrayList<PoseTriplet> moveForwardArray;
  private RamseteCommand moveForwardCommand;

  public ScoreThenBalance(
      DriveTrainSubsystem p_drivetrainSubsystem,
      ViennaPIDController p_pidController,
      ArmSubsystem p_armSubsystem,
      ScoringSubsystem p_scoringSubsystem,
      IntakeSubsystem p_intakeSubsystem,
      IntakeManager p_intakeManager,
      LEDSubsystem p_ledSubsystem,
      XboxController p_driverController, TelescopeSubsystem p_telescopeSubsystem) {

    moveForwardArray =
        new ArrayList<PoseTriplet>(
            Arrays.asList(new PoseTriplet(0, 0, 0), new PoseTriplet(2, 0, 0)));

    moveForwardCommand =
        AutonomousUtil.generateRamseteFromPoses(
            moveForwardArray,
            p_drivetrainSubsystem,
            TrajectoryConfigConstants.K_MAX_SPEED_FORWARD_CONFIG);

    addCommands(
        new ArmToAngleCommand(
            p_armSubsystem,
            p_pidController,
            p_driverController, p_telescopeSubsystem,
            ArmConstants.BACK_MID,
            true,
            false,
            p_ledSubsystem),
        new ScoringOpenCommand(p_scoringSubsystem, p_intakeManager).withTimeout(2),
        moveForwardCommand,
        new AutoBalanceCommand(p_drivetrainSubsystem));
  }
}
