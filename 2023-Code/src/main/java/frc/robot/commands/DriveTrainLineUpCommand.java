// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class DriveTrainLineUpCommand extends CommandBase {
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
  double setDistance;
  boolean save = false;
  double tx;
  boolean endCommand = false;
  double tx1;
  double tx2;
  double turnAngle1;
  double turnAngle2;
  int programNumber;
  double offSet;
  double tagDistanceScore = 1;
  double tagDistanceStation = 1.3;

  double firstTurnTune = 0.3;
  double firstStraightTune = 0.2;
  double secondTurnFlat = Math.PI / 4;
  double firstTxTune = 2.5;


  public DriveTrainLineUpCommand(DriveTrainSubsystem p_drivetrain, LimelightSubsystem p_limelight, int programNumber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = p_drivetrain;
    m_limelight = p_limelight;
    this.programNumber = programNumber;
    addRequirements(m_drivetrain, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("starting command \nsave between command runs = " + save);
    // System.out.println("lmotorposition " + lmotorPosition);
    // System.out.println("rmotorposition " + rmotorPosition);
    if (programNumber == 0){
      offSet = 0;
      m_limelight.setPipeline(0);
    }
    else if (programNumber == 1){
      offSet = 0.43;
      m_limelight.setPipeline(1);
    }
    else if (programNumber == 2){
      offSet = -0.43;
      m_limelight.setPipeline(2);
    }
    else if (programNumber == 3){
      offSet = 0.77;
      m_limelight.setPipeline(3);
    }
    else if (programNumber == 4){
      offSet = -0.77;
      m_limelight.setPipeline(4);
    }
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
          array[0] = array[0] + offSet;

          double slope = array[1] / array[0];
          theta = Math.atan(slope);
          actionCounter++;
          // System.out.println("ended step 0");
          turnLeft = theta >= 0;
          System.out.println("theta " + theta);
          System.out.println("step 0 ended");
        }
      }
    } else if (actionCounter == 1) {
      if (firstRun == 0) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
        // tx1 = Math.PI / 180 * m_limelight.getTx();
        // tx2 = Math.PI / 180 * m_limelight.getTx();
        // turnAngle1 = theta + tx1;
        // turnAngle2 = theta + tx2;
        // System.out.println("tx1 " + tx1 + " tx2 " + tx2);
        System.out.println("theta " + (turnAngle1 * 180 / Math.PI));
        // System.out.println("step one getting motor positions");
      }

      boolean stop = false;
      if (theta > 0) {

        stop = m_drivetrain.turnMotor(turnAngle1 - firstTurnTune, rmotorPosition, lmotorPosition);

      } else {

        stop = m_drivetrain.turnMotor(turnAngle2 + firstTurnTune, rmotorPosition, lmotorPosition);
        
      }

      firstRun++;
      if (stop) {
        actionCounter++;
        firstRun = 0;
        System.out.println("ended step 1");
      }

    } else if (actionCounter == 2) {
      // System.out.println("first run for 2 is " + firstRun);
      if (firstRun == 0) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
        System.out.println("side distance " + array[0]);
      }

      boolean stop = false;
      if (array[0] > 0) {
        //System.out.println("array0 " + array[0]);
        stop = m_drivetrain.setMotorPosition(array[0] + firstStraightTune, rmotorPosition, lmotorPosition);
      } else {
        //System.out.println("array0 " + array[0]);
        stop = m_drivetrain.setMotorPosition(array[0] * -1 + firstStraightTune, rmotorPosition, lmotorPosition);
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

      if (firstRun >= 15){

      if (turnLeft) {
        stop = m_drivetrain.turnMotor(-secondTurnFlat, rmotorPosition, lmotorPosition);
        System.out.println("step 3 tv " + m_limelight.getTv());
      } else {
        stop = m_drivetrain.turnMotor(secondTurnFlat, rmotorPosition, lmotorPosition);
        System.out.println("step 3 tv " + m_limelight.getTv());
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
          m_drivetrain.turnMotor(-0.05, rmotorPosition, lmotorPosition);
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
          m_drivetrain.turnMotor(0.05, rmotorPosition, lmotorPosition);
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
          tx -= firstTxTune;
          System.out.println("tx for 6 is " + tx);
        }
        else {
          tx += firstTxTune;
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
        if (programNumber == 0 || programNumber == 1 || programNumber == 2){
          setDistance = Math.abs(m_limelight.getDistance()) - tagDistanceScore;
        }
        else {
          setDistance = Math.abs(m_limelight.getDistance()) - tagDistanceStation;
        }
  
        System.out.println("distance " + m_limelight.getDistance());
        System.out.println("setpoint " + setDistance);
      }
      //setpoint logic needs some more work
      //later edit: I think it's fine but need to confirm
      //theres some messed up stuff with the distances so do some measurements
      //later edit: distances are still messed up from my memory but I need to test and confirm this as well
      boolean stop = false;

      if (firstRun >= 15){

      stop = m_drivetrain.setMotorPosition(setDistance, rmotorPosition, lmotorPosition);
      }

      firstRun++;
      if (stop) {
        actionCounter++;
        firstRun = 0;
        System.out.println("ended step 8");
      }
    } else if (actionCounter == 9) {

      if (firstRun == 45) {
        rmotorPosition = m_drivetrain.getR1Pos();
        lmotorPosition = m_drivetrain.getL1Pos();
      }

      boolean stop = false;

      if (firstRun == 45){
        tx = m_limelight.getTx();

        System.out.println("pre tx is " + tx);
        
        if (tx > 0 && tx > 1.5){
          tx -= 1.5;
        }
        else if (tx < 0 && tx < -1.5){
          tx += 1.5;
        }
        System.out.println("tx for 9 is " + tx);

        tx = tx * Math.PI / 180;
      }

      if (firstRun >= 45) {
        stop =
            m_drivetrain.turnMotor(tx, rmotorPosition, lmotorPosition);
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
