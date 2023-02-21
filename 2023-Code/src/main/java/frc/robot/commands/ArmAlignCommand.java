// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.ViennaPIDController;

public class ArmAlignCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private LimelightSubsystem m_limelightSubsystem;
  private ViennaPIDController m_pidController;
  private double m_targetAngle;
  private double voltage;

  /** Creates a new ArmToAngleCommand. */
  public ArmAlignCommand(
      ArmSubsystem p_armSubsystem, ViennaPIDController p_pidController, LimelightSubsystem p_limelightSubsystem) {
    m_armSubsystem = p_armSubsystem;
    m_pidController = p_pidController;
    m_limelightSubsystem = p_limelightSubsystem;
    addRequirements(m_armSubsystem, m_limelightSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_targetAngle = Math.toDegrees(m_limelightSubsystem.findArmAngle()) * Constants.ArmConstants.ENCODER_TO_DEGREES;
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
