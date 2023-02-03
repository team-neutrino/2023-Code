// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class DriveTrainDriveFowardCommand extends CommandBase {
  /** Creates a new DriveTrainDriveFowardCommand. */
  DriveTrainSubsystem m_drivetrain;

  LimelightSubsystem m_limelight;
  double theta;
  int actionCounter = 0;
  int ID;
  double array[];
  int firstRun = 0;
  double rmotorPosition;
  double lmotorPosition;
  int newID = -10;
  boolean turnLeft = false;
  double limelightAdjust;
  double setPoint;
  boolean save = false;

  public DriveTrainDriveFowardCommand(
      DriveTrainSubsystem p_drivetrain, LimelightSubsystem p_limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = p_drivetrain;
    m_limelight = p_limelight;
    addRequirements(m_drivetrain, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("starting command /n save between command runs = " + save);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (actionCounter == 0) {

      if (m_limelight.getTv()) {
        ID = m_limelight.getID();
        array = m_limelight.parseJson();
        if (array.length != 0) {
          if (array[2] <= 0) {
            array[0] = array[0] * -1;
          }
          double slope = array[1] / array[0];
          theta = Math.atan(slope);
          actionCounter++;
        }
      }
    } else if (actionCounter == 1) {

      if (firstRun == 0) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
      }

      boolean stop = false;

      if (theta > 0) {
        stop = m_drivetrain.turnMotor(theta - 0.25, rmotorPosition, lmotorPosition);
      } else {
        stop = m_drivetrain.turnMotor(theta + 0.25, rmotorPosition, lmotorPosition);
      }

      if (stop) {
        actionCounter++;
      }
      firstRun++;
    } else if (actionCounter == 2) {

      if (firstRun == 0) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
      }

      boolean stop = false;

      if (array[0] > 0) {
        stop = m_drivetrain.setMotorPosition(array[0], rmotorPosition, lmotorPosition);
      } else {
        stop = m_drivetrain.setMotorPosition(array[0] * -1, rmotorPosition, lmotorPosition);
      }

      if (stop) {
        actionCounter++;
        firstRun = 0;
      }
      firstRun++;
    } else if (actionCounter == 3) {

      if (firstRun == 0) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL2Pos();
        turnLeft = theta >= 0;
      }

      boolean stop = false;

      if (turnLeft) {
        stop = m_drivetrain.turnMotor(-Math.PI / 2, rmotorPosition, lmotorPosition);
      } else {
        stop = m_drivetrain.turnMotor(Math.PI / 2, rmotorPosition, lmotorPosition);
      }

      if (stop) {
        actionCounter++;
        firstRun = 0;
      }
      firstRun++;
    } else if (actionCounter == 4) {

      if (firstRun == 0) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
      }

      boolean stop = false;

      if (turnLeft) {

        if (m_limelight.getTv() == false || ID != newID) {
          stop = m_drivetrain.turnMotor(-0.01, rmotorPosition, lmotorPosition);
          newID = m_limelight.getID();
        }

        /*
        while (m_limelight.getTv() == false || ID != newID) {
          m_drivetrain.turnMotor(-0.01);
          newID = m_limelight.getID();
        }
        */
      } else {

        if (m_limelight.getTv() == false || ID != newID) {
          stop = m_drivetrain.turnMotor(0.01, rmotorPosition, lmotorPosition);
          newID = m_limelight.getID();
        }
        /*
        while (m_limelight.getTv() == false || ID != newID) {
          m_drivetrain.turnMotor(0.01);
          newID = m_limelight.getID();
        }
        */
      }

      if (stop) {
        actionCounter++;
        firstRun = 0;
      }
      firstRun++;
    } else if (actionCounter == 5) {

      boolean stop = false;

      if (firstRun == 0) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
        limelightAdjust = m_limelight.getTx() * Math.PI / 180;
      }

      if (limelightAdjust > 0) {
        limelightAdjust = limelightAdjust - 0.1;
      } else {
        limelightAdjust = limelightAdjust + 0.1;
      }

      stop = m_drivetrain.turnMotor(limelightAdjust, rmotorPosition, lmotorPosition);

      if (stop) {
        actionCounter++;
        firstRun = 0;
      }
      firstRun++;
    } else if (actionCounter == 6) {

      if (firstRun == 0) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
      }

      boolean stop = false;

      if (firstRun == 25) {
        stop =
            m_drivetrain.turnMotor(
                m_limelight.getTx() * Math.PI / 180, rmotorPosition, lmotorPosition);
      }

      if (stop) {
        actionCounter++;
        firstRun = 0;
      }
      firstRun++;
    } else if (actionCounter == 7) {

      if (firstRun == 0) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
      }

      boolean stop = false;

      if (firstRun == 15) {
        stop =
            m_drivetrain.turnMotor(
                m_limelight.getTx() * Math.PI / 180 / 2, rmotorPosition, lmotorPosition);
      }

      if (stop) {
        actionCounter++;
        firstRun = 0;
      }
      firstRun++;
    } else if (actionCounter == 8) {

      if (firstRun == 0) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
        array = m_limelight.parseJson();
        setPoint = array[1] + 1;
      }

      boolean stop = false;

      stop = m_drivetrain.setMotorPosition(setPoint * -1, rmotorPosition, lmotorPosition);

      if (stop) {
        actionCounter++;
        firstRun = 0;
      }
      firstRun++;
    } else if (actionCounter == 9) {

      if (firstRun == 0) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
      }

      boolean stop = false;

      if (firstRun == 25) {
        stop =
            m_drivetrain.turnMotor(
                m_limelight.getTx() * Math.PI / 180 / 2, rmotorPosition, lmotorPosition);
      }

      if (stop) {
        actionCounter++;
        firstRun = 0;
      }
      firstRun++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    save = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
