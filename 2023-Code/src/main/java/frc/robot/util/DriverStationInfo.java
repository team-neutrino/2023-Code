package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriverStationInfo {

  public DriverStationInfo() {}

  public static double getMatchTime() {
    return DriverStation.getMatchTime();
  }

  public static Alliance getAlliance() {
    return DriverStation.getAlliance();
  }
}
