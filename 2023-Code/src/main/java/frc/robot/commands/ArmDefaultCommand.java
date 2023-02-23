// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmDefaultCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;

  public ArmDefaultCommand(ArmSubsystem p_armSubsystem) {
    m_armSubsystem = p_armSubsystem;
    addRequirements(m_armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // m_armSubsystem.smartSet(Constants.ArmConstants.FORWARD_MID);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
