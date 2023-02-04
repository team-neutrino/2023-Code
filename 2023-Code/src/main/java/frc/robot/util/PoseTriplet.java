// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

public class PoseTriplet {
  private final double m_coord1;
  private final double m_coord2;
  private final double m_angle;

  public PoseTriplet(double p_coord1, double p_coord2, double p_angle) {
    m_coord1 = p_coord1;
    m_coord2 = p_coord2;
    m_angle = p_angle;
  }

  public double getCoord1() {
    return m_coord1;
  }

  public double getCoord2() {
    return m_coord2;
  }

  public double getAngle() {
    return m_angle;
  }
}
