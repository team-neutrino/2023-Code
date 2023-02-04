// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.ViennaPIDController;

public class ArmToAngleCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private ViennaPIDController m_pidController;
  private double m_targetAngle;
  private double voltage;

  /** Creates a new ArmToAngleCommand. */
  public ArmToAngleCommand(
      ArmSubsystem p_armSubsystem, ViennaPIDController p_pidController, double p_targetAngle) {
    m_armSubsystem = p_armSubsystem;
    m_pidController = p_pidController;
    addRequirements(m_armSubsystem);
    m_targetAngle = p_targetAngle;
  }

  @Override
  public void initialize() {
    // m_armSubsystem.setReference(m_angle);

  }

  @Override
  public void execute() {
    voltage = m_pidController.run(m_armSubsystem.getAbsolutePosition(), m_targetAngle);
    m_armSubsystem.set(voltage);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
