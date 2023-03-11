// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveTrainDefaultCommand extends CommandBase {
  DriveTrainSubsystem m_drivetrainSubsystem;
  Joystick m_leftJoystick;
  Joystick m_rightJoystick;

  /** Creates a new DriveCommand. */
  // public DriveTrainDefaultCommand(
  //     DriveTrainSubsystem p_drivetrainSubsystem,
  //     Joystick p_leftJoystick,
  //     Joystick p_rightJoystick) {

  //   m_drivetrainSubsystem = p_drivetrainSubsystem;
  //   m_leftJoystick = p_leftJoystick;
  //   m_rightJoystick = p_rightJoystick;
  //   addRequirements(m_drivetrainSubsystem);
  // }

  public DriveTrainDefaultCommand(
      SubsystemContainer p_subsystemContainer, Joystick p_leftJoystick, Joystick p_rightJoystick) {
    m_drivetrainSubsystem = p_subsystemContainer.getDriveTrainSubsystem();
    m_leftJoystick = p_leftJoystick;
    m_rightJoystick = p_rightJoystick;
    addRequirements(m_drivetrainSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_drivetrainSubsystem.setMotors(m_leftJoystick.getY() * -1, m_rightJoystick.getY() * -1);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
