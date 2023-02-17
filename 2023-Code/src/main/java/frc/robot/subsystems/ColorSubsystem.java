// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSubsystem extends SubsystemBase {

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private ColorMatch m_colorMatcher = new ColorMatch();
  private boolean m_isYellow;
  private boolean m_isPurple;

  private final Color C_Yellow = new Color(87, 146, 21);
  private final Color C_Purple = new Color(55, 102, 96);
  private final Color C_LogoPurple = new Color(56, 115, 83);
  private final Color C_Default = new Color(66, 125, 63);

  public ColorSubsystem() {
    m_colorMatcher.addColorMatch(C_Yellow);
    m_colorMatcher.addColorMatch(C_Purple);
    m_colorMatcher.addColorMatch(C_LogoPurple);
    m_colorMatcher.addColorMatch(C_Default);
  }

  @Override
  public void periodic() {
    Color detectedColor = getColor();
    m_isYellow = isYellow(detectedColor);
    m_isPurple = isPurple(detectedColor);
  }

  public boolean getIsYellow() {
    return m_isYellow;
  }

  public boolean getIsPurple() {
    return m_isPurple;
  }

  public Color getColor() {
    return m_colorSensor.getColor();
  }

  public String getPiece() {
    if (m_isYellow) {
      return "Cone";
    } else if (m_isPurple) {
      return "Cube";
    } else {
      return "No piece";
    }
  }

  public boolean isYellow(Color detectedColor) {
    ColorMatchResult matchResult = m_colorMatcher.matchClosestColor(detectedColor);
    return C_Yellow.equals(matchResult.color);
  }

  public boolean isPurple(Color detectedColor) {
    ColorMatchResult matchResult = m_colorMatcher.matchClosestColor(detectedColor);
    return C_Purple.equals(matchResult.color) || C_LogoPurple.equals(matchResult.color);
  }

  public boolean isDefault(Color detectedColor) {
    ColorMatchResult matchResult = m_colorMatcher.matchClosestColor(detectedColor);
    return C_Default.equals(matchResult.color);
  }
}
