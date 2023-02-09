// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  public AddressableLED m_addressableLED;
  public AddressableLEDBuffer m_LedBuffer;
  private boolean m_beamBreak;

  public void start() {}

  public void stop() {}

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_addressableLED = new AddressableLED(0);
    m_LedBuffer = new AddressableLEDBuffer(8);

    m_addressableLED.setLength(m_LedBuffer.getLength());
    m_addressableLED.setData(m_LedBuffer);
    m_addressableLED.start();
    m_LedBuffer.setRGB(0, 256, 0, 0);
  }

  public void setToPurple() {
    for (int i = 0; i < m_LedBuffer.getLength(); i++) {
      m_LedBuffer.setRGB(i, 162, 25, 255);
    }
  }

  public void setToYellow() {
    for (int i = 0; i < m_LedBuffer.getLength(); i++) {
      m_LedBuffer.setRGB(i, 255, 100, 0);
    }
  }

  public void setToOrange() {
    for (int i = 0; i < m_LedBuffer.getLength(); i++) {
      m_LedBuffer.setRGB(i, 255, 15, 0);
    }
  }

  public String getColor() {
    Color orange = new Color(255, 15, 0);
    Color purple = new Color(162, 25, 255);
    Color yellow = new Color(255, 100, 0);

    if (m_LedBuffer.getLED(1).equals(orange)) {
      return "Orange";
    } else if (m_LedBuffer.getLED(1).equals(purple)) {
      return "Purple";
    } else if (m_LedBuffer.getLED(1).equals(yellow)) {
      return "Yellow";
    } else {
      return "Cannot determine color";
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_addressableLED.setData(m_LedBuffer);
  }
}
