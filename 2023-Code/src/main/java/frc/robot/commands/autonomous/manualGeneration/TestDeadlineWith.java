// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.manualGeneration;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ScoringOpenCommand;
import frc.robot.commands.autonomous.TimerCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.IntakeManager;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestDeadlineWith extends SequentialCommandGroup {

  private void printTest() {
    System.out.println("hello this is probably in a simulation");
  }

  /** Creates a new TestDeadlineWith. */
  public TestDeadlineWith(ScoringSubsystem p_scoringSubsystem, IntakeSubsystem p_intakeSubsystem, IntakeManager p_intakeManager) {
    // // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(this::printTest),
        new ScoringOpenCommand(p_scoringSubsystem, p_intakeSubsystem, p_intakeManager).withTimeout(4),
        new InstantCommand(this::printTest));
  }
}
