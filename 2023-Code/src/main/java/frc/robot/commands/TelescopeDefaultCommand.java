// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeDefaultCommand extends CommandBase {

  private TelescopeSubsystem m_telescopeSubsystem;
  private ArmSubsystem m_armSubsystem;

  /** Creates a new TelescopeDefaultCommand. */
  public TelescopeDefaultCommand(
      TelescopeSubsystem p_telescopeSubsystem, ArmSubsystem p_armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_telescopeSubsystem = p_telescopeSubsystem;
    m_armSubsystem = p_armSubsystem;

    addRequirements(m_telescopeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double armEncoderPosition = m_armSubsystem.getArmEncoder().getAbsolutePosition() * 100;
    // if ((armEncoderPosition >= ArmConstants.FORWARD_FLOOR_ARM_ANGLE_LIMIT
    //         || (armEncoderPosition <= ArmConstants.FORWARD_ARM_HEIGHT_LIMIT
    //             && armEncoderPosition >= ArmConstants.BACKWARD_ARM_HEIGHT_LIMIT))
    //     && m_telescopeSubsystem.getTelescopingExtension() <= 0) {
    //   m_telescopeSubsystem.setTelescope(ArmConstants.TELESCOPE_HEIGHT_LIMIT_RETRACT_SPEED);
    // }
    m_telescopeSubsystem.retractTelescoping();
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
