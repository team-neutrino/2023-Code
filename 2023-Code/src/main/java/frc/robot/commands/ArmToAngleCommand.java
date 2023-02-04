// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmToAngleCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private double m_angle;

  /** Creates a new ArmToAngleCommand. */
  public ArmToAngleCommand(
      ArmSubsystem p_armSubsystem, IntakeSubsystem p_intakeSubsystem, double p_angle) {
    m_armSubsystem = p_armSubsystem;
    m_intakeSubsystem = p_intakeSubsystem;
    addRequirements(m_armSubsystem);
    m_angle = p_angle;
  }

  @Override
  public void initialize() {
    // I up this returns true if the updownsolenoid is retracted (so the intake is down)
    if (m_intakeSubsystem.getUpDownSolenoidValue()) m_armSubsystem.setReference(m_angle);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
