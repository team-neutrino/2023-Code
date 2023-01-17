package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndGameSubsystem extends SubsystemBase {
  Solenoid m_rightSolenoid =
      new Solenoid(
          PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.SOLENOIDENDGAMERIGHT);
  Solenoid m_leftSolenoid =
      new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.SOLENOIDENDGAMELEFT);

  public EndGameSubsystem() {}

  @Override
  public void periodic() {}

  // Changes the state of the Right Pistons
  public void toggleSolenoidRight() {
    m_rightSolenoid.toggle();
  }

  // Tells whether or not the pistons controlled by the Right solenoid is extending or not
  public boolean getSolenoidvalueRight() {
    return m_rightSolenoid.get();
  }

  // Changes the state of the Left Pistons
  public void toggleSolenoidLeft() {
    m_leftSolenoid.toggle();
  }

  // Tells whether or not the pistons controlled by the left solenoid is extending or not
  public boolean getSolenoidvalueLeft() {
    return m_leftSolenoid.get();
  }

  public void turnOffSolenoid() {
    m_rightSolenoid.close();
    m_leftSolenoid.close();
  }
}
