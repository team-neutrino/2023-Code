// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class DriveTrainAutoRotationCommand extends CommandBase {

    DriveTrainSubsystem m_drivetrain;
    LimelightSubsystem m_limelight;
    double tx;
    double rmotorPosition;
    double lmotorPosition;
    boolean stop = false;

    public DriveTrainAutoRotationCommand(DriveTrainSubsystem p_drivetrain, LimelightSubsystem p_limelight){
        m_drivetrain = p_drivetrain;
        m_limelight = p_limelight;
    }

    @Override
    public void initialize() {
        tx = m_limelight.getTx();
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
    }
  
    @Override
    public void execute() {
      stop = m_drivetrain.turnMotor(tx, rmotorPosition, lmotorPosition);
    }
  
    @Override
    public void end(boolean interrupted) {
        System.out.println("tx is " + m_limelight.getTx());
    }
  
    @Override
    public boolean isFinished() {
      return stop;
    }
    
}
