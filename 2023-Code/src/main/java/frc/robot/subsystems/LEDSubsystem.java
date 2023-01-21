// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  public AddressableLED m_addressableLED = new AddressableLED(9);
  public AddressableLEDBuffer m_LedBuffer = new AddressableLEDBuffer(60);

  public void setToPurple(){
    for (var i = 0; i < m_LedBuffer.getLength(); i++){
      m_LedBuffer.setRGB(i, 95, 0, 160);
    }
  }

  public void start(){}

  public void stop(){}

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_addressableLED.setLength(500);
    m_addressableLED.setData(m_LedBuffer);
    m_addressableLED.start();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setToPurple();

  }
}
