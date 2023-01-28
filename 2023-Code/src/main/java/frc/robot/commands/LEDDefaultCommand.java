package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class LEDDefaultCommand extends CommandBase {

  LEDSubsystem m_LedSubsystem;

  public LEDDefaultCommand(LEDSubsystem p_LEDSubsystem) {
    m_LedSubsystem = p_LEDSubsystem;
    addRequirements(m_LedSubsystem);
  } 

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_LedSubsystem.setToOrange();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
