// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  NetworkTable limelight;
  int cycle = 0;
  double[] camTran;
  //double[] camTran = getCamTran();

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    // global instance of the network table and gets the limelight table
    limelight = NetworkTableInstance.getDefault().getTable("limelight");

    // turns off LED
    limelight.getEntry("ledMode").setNumber(1);

  }

  // return whether or not the limlight sees a target
  public boolean getTv() {
    NetworkTableEntry tv = limelight.getEntry("tv");
    double validTarget = tv.getDouble(0.0);
    if (validTarget == 1) {
      return true;
    }
    return false;
  }

  // gets id of the april tag that is detected
  public double getID() {
    return limelight.getEntry("tid").getDouble(0.0);
  }

  // gets the x offest between the center of vision and the detected object
  public double getTx() {
    return limelight.getEntry("tx").getDouble(0.0);
  }

  // gets the y offest between the center of vision and the detected object
  public double getTy() {
    return limelight.getEntry("ty").getDouble(0.0);
  }

  public double testTransformDistance(double tx) {
    double distanceFromLimelight = Math.tan(tx);
    return distanceFromLimelight;
  }

  public double[] getCamTran(){
    return limelight.getEntry("camtran").getDoubleArray(new double[]{});
  }

  public void printCamTran(){
    if (cycle % 50 == 0){
      System.out.println("Translation X: " + camTran[0]);
      System.out.println("Translation Y: " + camTran[1]);
      System.out.println("Translation Z: " + camTran[2]);
      //System.out.println("Rotation Pitch: " + camTran[3]);
      //System.out.println("Rotation Yall: " + camTran[4]);
      //System.out.println("Rotation Roll: " + camTran[5]);
    }
  }

  @Override
  public void periodic() {
    cycle++;
    camTran = getCamTran();
    printCamTran();
    //System.out.println(test[0] + " " + test[1] + " " + test[2] + " " + test[3] + " " + test[4] + " " + test[5]);
    //System.out.println(test.length);
    //System.out.println(camTran[0]);
    //System.out.println(camTran[1]);
  }
}
