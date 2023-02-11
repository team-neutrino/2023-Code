package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.util.DriverStationInfo;

public class LEDPotentialDefaultCommand extends CommandBase {

  private LEDSubsystem m_LedSubsystem;
  private DriverStationInfo m_DriverStationInfo;

  public LEDPotentialDefaultCommand() {}

  @Override
  public void initialize() {
    if (m_DriverStationInfo.getAlliance().equals(Alliance.Red)) {
      m_LedSubsystem.setToRed();
    }
    if (m_DriverStationInfo.getAlliance().equals(Alliance.Blue)) {
      m_LedSubsystem.setToBlue();
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
