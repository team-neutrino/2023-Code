// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToAngleCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private double m_angle;

  /** Creates a new ArmToAngleCommand. */
  public ArmToAngleCommand(ArmSubsystem p_armSubsystem, double p_angle) {
    m_armSubsystem = p_armSubsystem;
    addRequirements(m_armSubsystem);
    m_angle = p_angle;
  }

  @Override
  public void initialize() {
    m_armSubsystem.setReference(m_angle);
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
