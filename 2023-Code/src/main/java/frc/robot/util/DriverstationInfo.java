package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public class DriverStationInfo {

  public DriverStationInfo() {}

  public double getMatchTime() {
    return DriverStation.getMatchTime();
  }
}