// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import java.util.concurrent.TimeUnit;

public class DriveTrainDriveFowardCommand extends CommandBase {
  /** Creates a new DriveTrainDriveFowardCommand. */
  DriveTrainSubsystem m_drivetrain;

  LimelightSubsystem m_limelight;
  double theta;

  public DriveTrainDriveFowardCommand(
      DriveTrainSubsystem p_drivetrain, LimelightSubsystem p_limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = p_drivetrain;
    m_limelight = p_limelight;
    addRequirements(m_drivetrain, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelight.getTv()) {
      int ID = m_limelight.getID();
      double[] array = m_limelight.parseJson();
      if (array.length != 0) {
        if (array[2] <= 0) {
          array[0] = array[0] * -1;
        }
        double slope = array[1] / array[0];
        theta = Math.atan(slope);

        if (theta > 0) {
          m_drivetrain.turnMotor(theta - 0.25);
        } else {
          m_drivetrain.turnMotor(theta + 0.25);
        }

        if (array[0] > 0) {
          m_drivetrain.setMotorPosition(array[0], array[0]);
        } else {
          m_drivetrain.setMotorPosition(array[0] * -1, array[0] * -1);
        }

        boolean turnLeft = theta >= 0;

        if (turnLeft) {
          m_drivetrain.turnMotor(-Math.PI / 2);
        } else {
          m_drivetrain.turnMotor(Math.PI / 2);
        }

        int newID = -10;

        if (turnLeft) {
          while (m_limelight.getTv() == false || ID != newID) {
            m_drivetrain.turnMotor(-0.01);
            newID = m_limelight.getID();
          }
        } else {
          while (m_limelight.getTv() == false || ID != newID) {
            m_drivetrain.turnMotor(0.01);
            newID = m_limelight.getID();
          }
        }

        double limelightAdjust = m_limelight.getTx() * Math.PI / 180;
        if (limelightAdjust > 0) {
          limelightAdjust = limelightAdjust - 0.1;
        } else {
          limelightAdjust = limelightAdjust + 0.1;
        }

        m_drivetrain.turnMotor(limelightAdjust);

        try {
          TimeUnit.MILLISECONDS.sleep(500);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }

        System.out.println("tx " + m_limelight.getTx());
        m_drivetrain.turnMotor(m_limelight.getTx() * Math.PI / 180);

        try {
          TimeUnit.MILLISECONDS.sleep(300);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
        System.out.println("tx is " + m_limelight.getTx());
        m_drivetrain.turnMotor(m_limelight.getTx() * Math.PI / 180 / 2);
        array = m_limelight.parseJson();
        double setPoint = array[1] + 1;
        System.out.println("setpoint is " + setPoint);
        m_drivetrain.setMotorPosition(setPoint * -1, setPoint * -1);

        try {
          TimeUnit.MILLISECONDS.sleep(500);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }

        m_drivetrain.turnMotor(m_limelight.getTx() * Math.PI / 180);
      }
    }
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
