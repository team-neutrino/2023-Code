// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ScoringSubsystem extends SubsystemBase {
  private ScoringSubsystem m_scoringSubsystem;

  public ScoringSubsystem(ScoringSubsystem p_scoringSubsystem) {
    m_scoringSubsystem = p_scoringSubsystem;
  }

  private Solenoid solenoid =
      new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.GRABBER);

  private DigitalInput m_beamBreak = new DigitalInput(Constants.DigitalConstants.GRABBER_BEAMBREAK);

  /** Creates a new PneumaticsSubsystem. */
  public ScoringSubsystem() {}

  public void toggleSolenoid() {
    solenoid.toggle();
  }

  public void closeScoring() {
    solenoid.set(false);
  }

  public void openScoring() {
    solenoid.set(true);
  }

  public boolean getSolenoidValue() {
    return solenoid.get();
  }

  public boolean detectedGamePiece() {
    return !m_beamBreak.get();
  }

  @Override
  public void periodic() {}
}
