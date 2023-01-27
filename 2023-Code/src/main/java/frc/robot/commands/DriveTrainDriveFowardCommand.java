// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class DriveTrainDriveFowardCommand extends CommandBase {
  /** Creates a new DriveTrainDriveFowardCommand. */
  DriveTrainSubsystem m_drivetrain;

  LimelightSubsystem m_limelight;
  double sideA;
  double sideB;
  double sideC;
  double gamma;
  double alpha;
  double theta;
  double sideL;

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
    System.out.println("test");
  }

  // Called every time the scheduler runs while the command is scheduled.

  // negative == left
  // positive == right
  @Override
  public void execute() {
    // System.out.println("running");
    if (m_limelight.getTv()){
      double[] array = m_limelight.parseJson();
      if (array.length != 0){
        
        //boolean tagOnLeft = array[0]>=0;
        System.out.println(array[0] + " " + array[1]);
        double slope = array[1] / array[0];
        theta = Math.atan(slope);
        
        System.out.println("slope is " + slope);
        System.out.println("theta is " + theta);
        System.out.println("tan of theta is " + Math.tan(theta));
        System.out.println("theta in degrees is " + theta * 180/Math.PI);
        m_drivetrain.turnMotor(theta - 0.3);
        
        System.out.println("set motor position " + (array[0] * -1));
        if (array[0] > 0){
        m_drivetrain.setMotorPosition(array[0], array[0]);
        }
        else {
          m_drivetrain.setMotorPosition(array[0] * -1, array[0] * -1);
        }

        boolean turnLeft = theta >= 0;

        System.out.println("running 90 degree turn");
        if(turnLeft) {
          System.out.println("turning left");
          m_drivetrain.turnMotor(-Math.PI / 2);
        }
        else {
          System.out.println("turning right");
          m_drivetrain.turnMotor(Math.PI / 2);
        }

        boolean wait = m_limelight.waitBoolean();
        

        System.out.println("ran wait command");
        System.out.println("tv is " + m_limelight.getTv());
        double[] array2 = m_limelight.parseJson();
        System.out.println("should be running limelight adjust");
        if (wait){
        if (array2.length != 0){
          System.out.println("limelight adjust");
        slope = array2[1] / array2[0];
        System.out.println(array2[0] + " " + array2[1]);
        theta = Math.atan(slope);
        System.out.println("theta limelight adjust " + theta * 180/Math.PI);
        m_drivetrain.turnMotor(m_limelight.getTx() * Math.PI/180);
      }
    }
    }
  }

    /* 
    sideA = m_limelight.getDistance();
    sideB = m_limelight.getStraightDistance();
    gamma = m_limelight.getTx() * Math.PI / 180;
    sideC = m_limelight.lawOfCosines(sideA, sideB, gamma);
    alpha = m_limelight.lawOfSines(sideA, sideC, gamma);
    // theta = Math.PI - alpha - gamma;
    sideL = sideA * Math.cos(theta);

    m_drivetrain.turnMotorRight(Math.PI / 2);
    */
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
