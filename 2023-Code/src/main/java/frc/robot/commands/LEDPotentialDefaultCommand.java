package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.util.DriverStationInfo;

public class LEDPotentialDefaultCommand extends CommandBase {

  private LEDSubsystem m_ledSubsystem;
  private DriverStationInfo m_driverStationInfo;

  public LEDPotentialDefaultCommand() {}

  @Override
  public void initialize() {
    if (m_driverStationInfo.getAlliance().equals(Alliance.Red)) {
      m_ledSubsystem.setToRed();
    } else if (m_driverStationInfo.getAlliance().equals(Alliance.Blue)) {
      m_ledSubsystem.setToBlue();
    } else {
      m_ledSubsystem.setToOrange();
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
