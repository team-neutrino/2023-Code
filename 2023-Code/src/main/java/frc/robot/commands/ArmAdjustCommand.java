// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.ViennaPIDController;

public class ArmAdjustCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private XboxController m_driverController;
  double voltage;

  public ArmAdjustCommand(ArmSubsystem p_armSubsystem, XboxController p_driverController) {
    m_armSubsystem = p_armSubsystem;
    m_driverController = p_driverController;
    voltage = 0;

    addRequirements(m_armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(m_driverController.getRightY() < -0.1) {
      voltage = -.2;
    }
    else if(m_driverController.getRightY() > 0.1) {
      voltage = .2;
    }
    else {
      voltage = 0;
    }
    m_armSubsystem.smartSet(voltage);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
