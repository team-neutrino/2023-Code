// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ScoringSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BasicAutonomousCommand extends SequentialCommandGroup {

  // private ScoringSubsystem m_scoringSubsystem;

  /** Creates a new BasicAutonomousCommand. */
  public BasicAutonomousCommand(ScoringSubsystem p_scoringSubsystem) {
    // m_scoringSubsystem = p_scoringSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    SequentialCommandGroup testCommandGroup =
        new SequentialCommandGroup(
            new InstantCommand(p_scoringSubsystem::toggleSolenoid),
            new WaitCommand(0.5),
            new InstantCommand(p_scoringSubsystem::toggleSolenoid));

    addCommands(testCommandGroup.repeatedly());
  }
}
