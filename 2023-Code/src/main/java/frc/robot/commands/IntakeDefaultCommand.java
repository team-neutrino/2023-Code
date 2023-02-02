// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This default command serves the sole purpose of ensuring that when the button is NOT pressed, the
 * intake is up and stopped.
 */
public class IntakeDefaultCommand extends CommandBase {
  /** An object of the intake subsystem. */
  IntakeSubsystem m_intakeSubsystem;

  /** Constructor, creates a new IntakeDefaultCommand. */
  public IntakeDefaultCommand(IntakeSubsystem subsystem) {
    m_intakeSubsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * This command's purpose is to ensure that the intake is up and not running when the button isn't
   * pressed.
   */
  @Override
  public void execute() {
    m_intakeSubsystem.stopIntake();
<<<<<<< Updated upstream
    // in case we're holding a game piece, we'll want to keep holding it
=======
    // in case we're holding a game piece, we want to keep a hold of it
>>>>>>> Stashed changes
    m_intakeSubsystem.squeeze();
    m_intakeSubsystem.setIntakeUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
