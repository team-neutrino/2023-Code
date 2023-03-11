// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.PWMConstants;
import frc.robot.Constants.UtilConstants;
import frc.robot.util.LEDColor;

public class LEDSubsystem extends SubsystemBase {

  public AddressableLED m_addressableLED;
  public AddressableLEDBuffer m_LEDBuffer;
  private Timer timer = new Timer();

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_addressableLED = new AddressableLED(PWMConstants.LED_PORT);
    m_LEDBuffer = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);

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
    setToColor(LEDColor.PURPLE.getR(), LEDColor.PURPLE.getG(), LEDColor.PURPLE.getB());
  }

  public void setToYellow() {
    setToColor(LEDColor.YELLOW.getR(), LEDColor.YELLOW.getG(), LEDColor.YELLOW.getB());
  }

  public void setToOrange() {
    setToColor(LEDColor.ORANGE.getR(), LEDColor.ORANGE.getG(), LEDColor.ORANGE.getB());
  }

  public void setToRed() {
    setToColor(LEDColor.RED.getR(), LEDColor.RED.getG(), LEDColor.RED.getB());
  }

  public void setToBlue() {
    setToColor(LEDColor.BLUE.getR(), LEDColor.BLUE.getG(), LEDColor.BLUE.getB());
  }

  public LEDColor getColor() {
    // use of 1 as index is arbitrary
    if (m_LEDBuffer.getLED(1).equals(LEDConstants.ORANGE)) {
      return LEDColor.ORANGE;
    } else if (m_LEDBuffer.getLED(1).equals(LEDConstants.PURPLE)) {
      return LEDColor.PURPLE;
    } else if (m_LEDBuffer.getLED(1).equals(LEDConstants.YELLOW)) {
      return LEDColor.YELLOW;
    } else if (m_LEDBuffer.getLED(1).equals(LEDConstants.RED)) {
      return LEDColor.RED;
    } else if (m_LEDBuffer.getLED(1).equals(LEDConstants.BLUE)) {
      return LEDColor.BLUE;
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
  }
}
