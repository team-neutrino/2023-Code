// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.TelescopeSubsystem;

public class ReverseTelescope extends CommandBase {
  TelescopeSubsystem m_telescopeSubsystem;
  Timer m_timer = new Timer();
  double m_time;
  /** Creates a new ReverseTelescope. */
  public ReverseTelescope(SubsystemContainer p_subsystemContainer, double p_time) {
    m_telescopeSubsystem = p_subsystemContainer.getTelescopeSubsystem();
    m_time = p_time;
    addRequirements(m_telescopeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_telescopeSubsystem.retractTelescoping();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.restart();
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.get() >= m_time) {
      return true;
    }
    return false;
  }
}
