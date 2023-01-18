// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ScoringSubsystem extends SubsystemBase {

  private Solenoid solenoid =
      new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.GRABBER);

  /** Creates a new PneumaticsSubsystem. */
  public ScoringSubsystem() {}

  public void toggleSolenoid() {
    solenoid.toggle();
  }

  public void setSolenoidExtend() {
    solenoid.set(true);
  }

  public void setSolenoidRetract() {
    solenoid.set(false);
  }

  public boolean getSolenoidValue() {
    return solenoid.get();
  }

  @Override
  public void periodic() {}
}