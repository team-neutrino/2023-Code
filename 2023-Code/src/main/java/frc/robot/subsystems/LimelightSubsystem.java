// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/**
 * @author Neutrino Controls
 */
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  NetworkTable limelight;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    /** global instance of the network table and gets the limelight table */
    limelight = NetworkTableInstance.getDefault().getTable("limelight");

    /** turns off LED */
    limelight.getEntry("ledMode").setNumber(1);
  }

  /** returns whether the limelight sees a target */
  public boolean getTv() {
    NetworkTableEntry tv = limelight.getEntry("tv");
    if (tv.getDouble(0.0) == 1) {
      return true;
    }
    return false;
  }

  /** gets ID of the AprilTag that is detected */
  public double getID() {
    return limelight.getEntry("tid").getDouble(0.0);
  }

  /** gets the x offset between the center of vision and the detected object */
  public double getTx() {
    return limelight.getEntry("tx").getDouble(0.0);
  }

  /** gets the y offset between the center of vision and the detected object */
  public double getTy() {
    return limelight.getEntry("ty").getDouble(0.0);
  }

  @Override
  public void periodic() {}
}
