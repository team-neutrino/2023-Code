// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveTrainDefaultCommand extends CommandBase {
  DriveTrainSubsystem m_driveTrainSubsystem;
  Joystick m_leftJoystick;
  Joystick m_rightJoystick;

  /** Creates a new DriveCommand. */
  public DriveTrainDefaultCommand(
      DriveTrainSubsystem subsystem, Joystick p_leftJoystick, Joystick p_rightJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_driveTrainSubsystem = subsystem;
    m_leftJoystick = p_leftJoystick;
    m_rightJoystick = p_rightJoystick;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrainSubsystem.setMotors(DriveTrainSubsystem.linearAccel(m_leftJoystick.getY()), DriveTrainSubsystem.linearAccel(m_rightJoystick.getY()));
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
