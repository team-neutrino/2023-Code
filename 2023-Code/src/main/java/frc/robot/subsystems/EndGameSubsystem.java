package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndGameSubsystem extends SubsystemBase {
  Solenoid m_forwardSolenoid =
      new Solenoid(
          PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.SOLENOIDENDGAMEFORWARD);
  Solenoid m_backwardSolenoid =
      new Solenoid(
          PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.SOLENOIDENDGAMEBACKWARD);

  public EndGameSubsystem() {}

  @Override
  public void periodic() {}

  // Changes the state of the Right Pistons
  public void toggleSolenoidForward() {
    m_forwardSolenoid.toggle();
  }

  // Tells whether or not the pistons controlled by the Right solenoid is extending or not
  public boolean getSolenoidvalueForward() {
    return m_forwardSolenoid.get();
  }

  // Changes the state of the Left Pistons
  public void toggleSolenoidBackward() {
    m_backwardSolenoid.toggle();
  }

  // Tells whether or not the pistons controlled by the left solenoid is extending or not
  public boolean getSolenoidvalueBackward() {
    return m_backwardSolenoid.get();
  }
}
