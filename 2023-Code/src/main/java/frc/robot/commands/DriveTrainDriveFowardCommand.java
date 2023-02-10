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
  double tx;
  boolean endCommand = false;
  boolean txSeen = false;

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
    System.out.println("starting command \nsave between command runs = " + save);
    // System.out.println("lmotorposition " + lmotorPosition);
    // System.out.println("rmotorposition " + rmotorPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //DO THE MATH
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
          // System.out.println("ended step 0");
          turnLeft = theta >= 0;
          System.out.println("theta " + theta);
          System.out.println("step 0 ended");
        }
      }
    //STEP ONE
    } else if (actionCounter == 1) {
      //DO THE MATH AGAIN
      if (firstRun == 0) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
        // System.out.println("step one getting motor positions");
      }

      boolean stop = false;
      //TURNS THETA
      if (theta > 0) {
        if (theta >= 0.95){
        stop = m_drivetrain.turnMotor(theta - 0.3, rmotorPosition, lmotorPosition);
        }
      } else {
        if (theta <= -0.95){
        stop = m_drivetrain.turnMotor(theta + 0.3, rmotorPosition, lmotorPosition);
        }
      }

      firstRun++;
      if (stop) {
        actionCounter++;
        firstRun = 0;
        System.out.println("ended step 1");
      }

    //the same as step one?!?!?!??!?!
    } else if (actionCounter == 2) {
      // System.out.println("first run for 2 is " + firstRun);
      if (firstRun == 0) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
      }

      boolean stop = false;
      //drives forward the good distance perpendicular
      if (array[0] > 0) {
        //System.out.println("array0 " + array[0]);
        stop = m_drivetrain.setMotorPosition(array[0], rmotorPosition, lmotorPosition);
      } else {
        //System.out.println("array0 " + array[0]);
        stop = m_drivetrain.setMotorPosition(array[0] * -1, rmotorPosition, lmotorPosition);
      }

      firstRun++;
      if (stop) {
        actionCounter++;
        firstRun = 0;
        System.out.println("ended step 2");
        // System.out.println("firstrun after step 2 " + firstRun);
      }
    } else if (actionCounter == 3) {

      if (firstRun == 0) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL2Pos();
        turnLeft = theta >= 0;
        System.out.println(turnLeft);
        System.out.println("first run for 3 ");
      }

      boolean stop = false;

      if (turnLeft) {
        stop = m_drivetrain.turnMotor(-Math.PI / 4, rmotorPosition, lmotorPosition);
        System.out.println("step 3 tv " + m_limelight.getTv());
        if (m_limelight.getTv() == true){
          txSeen = true;
        }
      } else {
        stop = m_drivetrain.turnMotor(Math.PI / 4, rmotorPosition, lmotorPosition);
        System.out.println("step 3 tv " + m_limelight.getTv());
        if (m_limelight.getTv() == true){
          txSeen = true;
        }
      }

      firstRun++;
      if (stop) {
        actionCounter++;
        firstRun = 0;
        turnLeft = false;
        System.out.println("ended step 3");
        // System.out.println("first run after step 3 " + firstRun);
      }
    } else if (actionCounter == 4) {

      if (firstRun == 0) {
        turnLeft = theta >= 0;
      }

      rmotorPosition = m_drivetrain.getR1Pos();
      lmotorPosition = m_drivetrain.getL1Pos();
      boolean tv = false;
      boolean stop = false;

      newID = m_limelight.getID();

      if (firstRun % 20 == 0){
        System.out.println("tv is " + m_limelight.getTv());
        tv = m_limelight.getTv();
        System.out.println("compared id " + ID);
        System.out.println("current ID " + newID);
      }
      //boolean txTurnLeft = m_limelight.getTx() <=  0;

      if (turnLeft) {

        if (m_limelight.getTv() == false || ID != newID) {
          stop = m_drivetrain.turnMotor(-0.05, rmotorPosition, lmotorPosition);
          //System.out.println("stop is " + stop);
          //System.out.println("tv is --- " + m_limelight.getTv());
          //System.out.println("compared ID " + ID);
          //System.out.println("current id " + newID);
        }

        /*
        while (m_limelight.getTv() == false || ID != newID) {
          m_drivetrain.turnMotor(-0.01);
          newID = m_limelight.getID();
        }
        */
      } 

      else {

        if (m_limelight.getTv() == false || ID != newID) {
          stop = m_drivetrain.turnMotor(0.05, rmotorPosition, lmotorPosition);
          //System.out.println("tv is --- " + m_limelight.getTv());
          //System.out.println("compared ID " + ID);
          //System.out.println("current id " + newID);
        }
        /*
        while (m_limelight.getTv() == false || ID != newID) {
          m_drivetrain.turnMotor(0.01);
          newID = m_limelight.getID();
        }
        */
      }

      firstRun++;
      if (tv == true && ID == newID) {
        actionCounter++;
        firstRun = 0;
        System.out.println("ended step 4");
      }
    } else if (actionCounter == 5) {

      boolean stop = false;

      if (firstRun == 0) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
        limelightAdjust = m_limelight.getTx() * Math.PI / 180;
        System.out.println(m_limelight.getTx());
      }

      /* 
      if (limelightAdjust > 0) {
        limelightAdjust = limelightAdjust;
      } else {
        limelightAdjust = limelightAdjust;
      }
      */

      stop = m_drivetrain.turnMotor(limelightAdjust, rmotorPosition, lmotorPosition);
      System.out.println("step 5 tx " + limelightAdjust);

      firstRun++;
      if (stop) {
        actionCounter++;
        firstRun = 0;
        System.out.println("ended step 5");
      }
    } else if (actionCounter == 6) {

      if (firstRun == 20) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
      }

      boolean stop = false;

      if (firstRun == 45){
        tx = m_limelight.getTx();
        if (tx > 0){
          tx -= 2.5;
          System.out.println("tx for 6 is " + tx);
        }
        else {
          tx += 2.5;
          System.out.println("tx for 6 is " + tx);
        }
      }

      if (firstRun >= 45) {
        stop =
            m_drivetrain.turnMotor(
                tx * Math.PI / 180, rmotorPosition, lmotorPosition);
      }

      firstRun++;
      if (stop) {
        actionCounter++;
        firstRun = 0;
        System.out.println("ended step 6");
        tx = 0.0;
      }
    } else if (actionCounter == 7) {

      if (firstRun == 25) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
      }

      boolean stop = true;

      if (firstRun == 25){
        tx = m_limelight.getTx();
      }

      if (firstRun >= 25) {
        stop =
            m_drivetrain.turnMotor(
                tx * Math.PI / 180, rmotorPosition, lmotorPosition);
                System.out.println("tx for 7 is " + m_limelight.getTx());
      }

      firstRun++;
      if (stop) {
        actionCounter++;
        firstRun = 0;
        System.out.println("ended step 7");
      }
    } else if (actionCounter == 8) {

      if (firstRun == 0) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
        array = m_limelight.parseJson();
        setPoint = array[1] + 1;
      }

      boolean stop = false;

      stop = m_drivetrain.setMotorPosition(setPoint * -1, rmotorPosition, lmotorPosition);

      firstRun++;
      if (stop) {
        actionCounter++;
        firstRun = 0;
      }
    } else if (actionCounter == 9) {

      if (firstRun == 45) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
      }

      boolean stop = false;

      if (firstRun == 45){
        tx = m_limelight.getTx();
        if (tx <= 0.5 && tx >= -0.5){
          tx = 0;
        }

        if (tx > 0){
          tx -= 1.5;
        }
        else {
          tx += 1.5;
        }
        System.out.println("tx for 9 is" + tx);
      }

      if (firstRun >= 45) {
        stop =
            m_drivetrain.turnMotor(
                tx * Math.PI / 180, rmotorPosition, lmotorPosition);
                //System.out.println("tx for 9 is " + tx);
                System.out.println("stop is " + stop);
      }

      firstRun++;
      if (stop) {
        actionCounter++;
        firstRun = 0;
        endCommand = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("command ended");
    save = true;
    actionCounter = 0;
    firstRun = 0;
    endCommand = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
  }
}
