package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndGameSubsystem extends SubsystemBase {
  Solenoid m_frontSolenoid =
      new Solenoid(
          PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.SOLENOID_FRONT);
  Solenoid m_backSolenoid =
      new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.SOLENOID_BACK);

  public EndGameSubsystem() {}

  @Override
  public void periodic() {}

  // Changes the state of the Front Pistons
  public void toggleSolenoidFront() {
    m_frontSolenoid.toggle();
  }

  // Tells whether or not the pistons controlled by the front solenoid is extending or not
  public boolean getSolenoidvalueFront() {
    return m_frontSolenoid.get();
  }

  // Changes the state of the Back Pistons
  public void toggleSolenoidBack() {
    m_backSolenoid.toggle();
  }

  // Tells whether or not the pistons controlled by the back solenoid is extending or not
  public boolean getSolenoidvalueBack() {
    return m_backSolenoid.get();
  }
}
