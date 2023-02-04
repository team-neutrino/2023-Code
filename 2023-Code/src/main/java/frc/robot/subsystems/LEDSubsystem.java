// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LEDColor;

public class LEDSubsystem extends SubsystemBase {

  public AddressableLED m_addressableLED;
  public AddressableLEDBuffer m_LEDBuffer;
  private Timer timer = new Timer();

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_addressableLED = new AddressableLED(0);
    m_LEDBuffer = new AddressableLEDBuffer(10);

    m_addressableLED.setLength(m_LEDBuffer.getLength());
    m_addressableLED.setData(m_LEDBuffer);
    m_addressableLED.start();
    setToOrange();
    timer.start();
  }

  private void setToColor(int r, int g, int b) {
    for (int i = 0; i < m_LEDBuffer.getLength(); i++) {
      m_LEDBuffer.setRGB(i, r, g, b);
    }
  }

  public void setToPurple() {
    setToColor(210, 25, 210);
  }

  public void setToYellow() {
    setToColor(255, 100, 0);
  }

  public void setToOrange() {
    setToColor(255, 15, 0);
  }

  public LEDColor getColor() {
    Color orange = new Color(255, 15, 0);
    Color purple = new Color(162, 25, 255);
    Color yellow = new Color(255, 100, 0);

    if (m_LEDBuffer.getLED(1).equals(orange)) {
      return LEDColor.ORANGE;
    } else if (m_LEDBuffer.getLED(1).equals(purple)) {
      return LEDColor.PURPLE;
    } else if (m_LEDBuffer.getLED(1).equals(yellow)) {
      return LEDColor.YELLOW;
    } else {
      return LEDColor.INDETERMINATE;
    }
  }

  private void sarahStrobe() {
    double timeConst = Math.PI;
    int r = (int) Math.round(126 * Math.cos(timeConst / 4 * timer.get()) + 126);
    int g = (int) Math.round(126 * Math.cos(timeConst / 8 * timer.get()) + 126);
    int b = (int) Math.round(126 * Math.sin(timeConst / 2 * timer.get()) + 126);
    for (int i = 0; i < m_LEDBuffer.getLength(); ++i) setToColor(r, g, b);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_addressableLED.setData(m_LEDBuffer);
    sarahStrobe();
  }
}
