// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.UtilConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetNavXCommand extends SequentialCommandGroup {
  
  private DriveTrainSubsystem m_drivetrainSubsystem;

  private AHRS navX;

  /** Creates a new ResetNavXCommand. */
  public ResetNavXCommand(DriveTrainSubsystem p_drivetrainSubsystem) {
    m_drivetrainSubsystem = p_drivetrainSubsystem;
    navX = m_drivetrainSubsystem.getNavX();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> System.out.println("yaw before reset: " + navX.getYaw())),
      new InstantCommand(navX::reset),
      new WaitCommand(UtilConstants.NAVX_RESET_WAIT_TIME),
      new InstantCommand(navX::zeroYaw),
      new InstantCommand(() -> System.out.println("yaw after reset: " + navX.getYaw() + '\n'))
    );
  }
}
