// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/**
 * @author Neutrino Controls
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveTrainDefaultCommand extends CommandBase {
  DriveTrainSubsystem m_driveTrainSubsystem;
  Joystick m_leftJoystick;
  Joystick m_rightJoystick;

  /**
   * @param p_DriveTrainSubsystem - The drivetrain subsystem
   * @param p_leftJoystick - Left joystick, moves the left wheels
   * @param p_rightJoystick - Right joystick, moves the right wheels
   */
  public DriveTrainDefaultCommand(
      DriveTrainSubsystem p_DriveTrainSubsystem,
      Joystick p_leftJoystick,
      Joystick p_rightJoystick) {

    /** Takes subsystem dependencies */
    m_driveTrainSubsystem = p_DriveTrainSubsystem;
    m_leftJoystick = p_leftJoystick;
    m_rightJoystick = p_rightJoystick;
    addRequirements(p_DriveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /** Sets the motors to the Y position of the Joysticks */
    m_driveTrainSubsystem.setMotors(m_leftJoystick.getY(), m_rightJoystick.getY());
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
