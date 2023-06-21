// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubsystemContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmToAngleCommand;
import frc.robot.commands.ScoringOpenCommand;
import frc.robot.util.IntakeManager;
import frc.robot.util.ViennaPIDController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class JustScore extends SequentialCommandGroup {
  /** Creates a new JustScore. */
  public JustScore(    SubsystemContainer p_subsystemContainer,
  ViennaPIDController p_pidController,
  IntakeManager p_intakeManager,
  XboxController p_driverController) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmToAngleCommand(
            p_subsystemContainer,
            p_pidController,
            p_driverController,
            ArmConstants.BACK_MID,
            true,
            false),
        new ScoringOpenCommand(p_subsystemContainer, p_intakeManager).withTimeout(.75),
        new ArmToAngleCommand(
                p_subsystemContainer, p_pidController, p_driverController, ArmConstants.FORWARD_MID)
            .withTimeout(1));
  }
}
