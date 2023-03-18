// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.KeyCombination;

public class KeyCombo extends CommandBase {
  JoystickButton m_button;
  Command m_defaultCommand;
  Command m_comboCommand;
  KeyCombination m_keyCombination;
  /** Creates a new KeyCombo. */
  public KeyCombo(
      JoystickButton p_button,
      Command p_defaultCommand,
      Command p_comboCommand,
      KeyCombination p_keyCombination) {
    p_button = m_button;
    p_defaultCommand = m_defaultCommand;
    p_comboCommand = m_comboCommand;
    p_keyCombination = m_keyCombination;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_keyCombination.whileTrueButtonCombination(m_button, m_defaultCommand, m_comboCommand);
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
