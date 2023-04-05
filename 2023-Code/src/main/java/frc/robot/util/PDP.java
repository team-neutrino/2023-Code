// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants;

public class PDP {

  private PowerDistribution m_PDP =
      new PowerDistribution(Constants.PDPConstants.PDP_CAN_ID, ModuleType.kCTRE);
  public PDP() {}

  public double getVoltage() {
    double voltage = m_PDP.getVoltage();
    return voltage;
  }

  public double getCurrent() {
    double totalCurrent = m_PDP.getTotalCurrent();
    return totalCurrent;
  }

  public double getPower() {
    double totalPower = m_PDP.getTotalPower();
    return totalPower;
  }

  public double getEnergy() {
    double totalEnergy = m_PDP.getTotalEnergy();
    return totalEnergy;
  }

  public double getChannelCurrent(int channel) {
    double current = m_PDP.getCurrent(channel);
    return current;
  }
}
