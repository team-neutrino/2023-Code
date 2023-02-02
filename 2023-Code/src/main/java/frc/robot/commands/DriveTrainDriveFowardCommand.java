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
  int actionCounter = 0;
  int ID;
  double array[];
  boolean motorStop = false;
  int firstRun1 = 0;
  int firstRun2 = 0;
  int firstRun3 = 0;
  int firstRun4 = 0;
  int firstRun5 = 0;
  double rmotorPosition;
  double lmotorPosition;
  int newID = -10;
  boolean turnLeft = false;
  double limelightAdjust;

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
    if (actionCounter == 0){

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
}
    else if (actionCounter == 1){

      if (firstRun1 == 0){
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
      }

      if (theta > 0) {
        motorStop = m_drivetrain.turnMotor(theta - 0.25, rmotorPosition, lmotorPosition);
      } else {
        motorStop = m_drivetrain.turnMotor(theta + 0.25, rmotorPosition, lmotorPosition);
      }

      if (motorStop){
        actionCounter++;
      }
      firstRun1++;
    }

    else if (actionCounter == 2){

      if (array[0] > 0) {
        motorStop = m_drivetrain.setMotorPosition(array[0], array[0]);
      } else {
        motorStop = m_drivetrain.setMotorPosition(array[0] * -1, array[0] * -1);
      }

      if (motorStop){
        actionCounter++;
      }
    }

    else if (actionCounter == 3){

      if (firstRun2 == 0){
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
      if (stop){
        actionCounter++;
      }
      firstRun2++;
    }

    else if (actionCounter == 4){

      if (firstRun3 == 0){
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
      }

      boolean stop = false;

      if (turnLeft) {

        if (m_limelight.getTv() == false || ID != newID){
        stop = m_drivetrain.turnMotor(-0.01, rmotorPosition, lmotorPosition);
        newID = m_limelight.getID();
        }

        /* 
        while (m_limelight.getTv() == false || ID != newID) {
          m_drivetrain.turnMotor(-0.01);
          newID = m_limelight.getID();
        }
        */
      } 
      
      else {

        if (m_limelight.getTv() == false || ID != newID){
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

      if (stop){
        actionCounter++;
      }
      firstRun3++;
    }

    else if (actionCounter == 5){

      boolean stop = false;

      if (firstRun4 == 0){
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
      firstRun4++;

      if (stop){
        actionCounter++;
      }
    }

    else if (actionCounter == 6){

      if (firstRun5 == 0){
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
      }    

      m_drivetrain.turnMotor(m_limelight.getTx() * Math.PI / 180);

    }



        try {
          TimeUnit.MILLISECONDS.sleep(500);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }


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
