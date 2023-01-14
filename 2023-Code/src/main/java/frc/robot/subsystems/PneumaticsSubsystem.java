// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/**
 * @author Neutrino Controls
 */
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSubsystem extends SubsystemBase {

  private Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  /** Either CTREPCM or REVPH, we use CTRE Initialization not required for single solenoids */
  private Solenoid solenoid =
      new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.CHANNEL_PCM);

  /** Creates a new PneumaticsSubsystem. */
  public PneumaticsSubsystem() {}

  /** Toggles solenoid */
  public void toggleSolenoid() {
    solenoid.toggle();
  }

  /** Checks if the compressor is enabled */
  public boolean isEnabled() {
    return compressor.isEnabled();
  }

  /** Turns on solenoid */
  public void setSolenoidOn() {
    System.out.println("setSolenoidOn() " + getSolenoidValue());
    solenoid.set(true);
  }

  /** Turns off solenoid */
  public void setSolenoidOff() {
    System.out.println("setSolenoidOff() " + getSolenoidValue());
    solenoid.set(false);
  }

  /** Returns solenoid value */
  public boolean getSolenoidValue() {
    return solenoid.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    compressor.enableDigital();
  }
}
