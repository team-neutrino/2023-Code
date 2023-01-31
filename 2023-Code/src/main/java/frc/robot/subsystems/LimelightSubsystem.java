// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.json.JSONArray;
import org.json.JSONObject;

public class LimelightSubsystem extends SubsystemBase {
  NetworkTable limelight;
  int cycle = 0;
  int cycle2 = 0;
  double P = 1.5;
  double LIMELIGHT_TO_METER_CONVERSION = 0.76189;
  double ULTRASONIC_TO_METER_CONVERSION = 1.23;
  AnalogPotentiometer m_distanceFinder = new AnalogPotentiometer(0, 6, 0);

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
  public int getID() {
    return (int) limelight.getEntry("tid").getDouble(0.0);
  }

  // gets the x offest between the center of vision and the detected object
  public double getTx() {
    return limelight.getEntry("tx").getDouble(0.0);
  }

  // gets the y offest between the center of vision and the detected object
  public double getTy() {
    return limelight.getEntry("ty").getDouble(0.0);
  }

  public double[] getCamTran() {
    return limelight.getEntry("camtran").getDoubleArray(new double[] {});
  }

  public double getDistance() {
    double[] camTran = getCamTran();
    return camTran[2] * LIMELIGHT_TO_METER_CONVERSION * -1;
  }

  public double getYaw() {
    double[] camTran = getCamTran();
    return camTran[4];
  }

  public double getX() {
    double[] camTran = getCamTran();
    return camTran[0];
  }

  public double getStraightDistance() {
    return m_distanceFinder.get() * ULTRASONIC_TO_METER_CONVERSION;
  }

  public double lawOfCosines(double sideX, double sideY, double theta) {
    return Math.sqrt(
        Math.pow(sideX, 2) + Math.pow(sideY, 2) - (2 * sideX * sideY * Math.cos(theta)));
  }

  public double lawOfSines(double sideY, double sideZ, double theta) {
    return Math.asin(sideY * Math.sin(theta) / sideZ);
  }

  /* is a print that access and prints the full array that is accessed when getting the camTran. Inert for right now,
   * may be used in the shuffleboard subsystem so left here at the wish of Cale.
   */
  public void printCamTran() {
    if (cycle % 40 == 0) {
      double[] camTran = getCamTran();
      double[] jsonOutput = parseJson();
      if (jsonOutput.length != 0)
        System.out.println("json file output: x: " + jsonOutput[0] + " z: " + jsonOutput[1]);
      // System.out.println(parseJson());
      // System.out.println("Translation X: " + camTran[0]);
      // System.out.println("Translation Y: " + camTran[1]);
      // System.out.println("Translation Z: " hhehaeaehaehaeaehahehaeaeh+ camTran[2] *
      // LIMELIGHT_TO_METER_CONVERSION * -1);
      // System.out.println("Rotation Pitch: " + camTran[3]);
      // System.out.println("Rotation Yaw: " + camTran[4]);
      // System.out.println("Rotation Roll: " + camTran[5]);
      // System.out.println("distance is " + m_distanceFinder.get() *
      // ULTRASONIC_TO_METER_CONVERSION);
    }
  }

  public double[] parseJson() {
    String jsonString = limelight.getEntry("json").getString("not found");
    JSONObject jsonObject = new JSONObject(jsonString);
    JSONArray fiducial = jsonObject.getJSONObject("Results").getJSONArray("Fiducial");
    // JSONArray x = array.getJSONArray();
    if (fiducial.length() == 0) {
      return new double[] {};
    }
    JSONObject jsonObject2 = fiducial.getJSONObject(0);
    JSONArray t6c_ts = jsonObject2.getJSONArray("t6c_ts");
    double x = t6c_ts.getDouble(0);
    double z = t6c_ts.getDouble(2);
    double y = t6c_ts.getDouble(1);
    // System.out.println("z is hghrfhusgrhgrhulg" + z);
    // double z = array.getJSONObject(0).getJSONArray("t6s_ts").getDouble(2);

    double[] values = {x, z, y};
    return values;
  }

  public boolean waitBoolean(){
    cycle2 = 5;
    System.out.println("this ran at least once");
    while (cycle2 < 10000000){
      cycle2++;
      if (cycle2 % 10000 == 0){
        System.out.println(cycle2);
      }
    if (cycle2 % 100000 == 0){
      System.out.println("returned true at 1");
      return true;
    }
  }
  System.out.println("returned true at 2");
  return true;

  }

  @Override
  public void periodic() {
    cycle++;
    //cycle2++;
    printCamTran();
  }
}
