package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public class DriverstationInfo {

  public DriverstationInfo() {}

  public double getMatchTime() {
    return DriverStation.getMatchTime();
  }
}
