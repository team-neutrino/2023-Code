// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.Limiter;
import frc.robot.util.ViennaPIDController;

public class ArmAdjustCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private XboxController m_driverController;
  private ViennaPIDController m_pidController;
  private double targetAngle;

  public ArmAdjustCommand(
      ArmSubsystem p_armSubsystem,
      XboxController p_driverController,
      ViennaPIDController p_pidController) {
    m_pidController = p_pidController;
    m_armSubsystem = p_armSubsystem;
    m_driverController = p_driverController;
    targetAngle = m_armSubsystem.getAbsolutePosition();

    addRequirements(m_armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double output;
    if (Math.abs(m_driverController.getRightY()) > ArmConstants.ARM_INPUT_DEADZONE) {
      output = m_driverController.getRightY() / ArmConstants.SCALE_QUOTIENT;
      targetAngle = m_armSubsystem.getAbsolutePosition();
    } else {
      int position = (int) m_armSubsystem.getAbsolutePosition();
      output = m_pidController.run(position, targetAngle);
    }
    m_armSubsystem.smartSet(output);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
