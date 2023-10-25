package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.TelescopeSubsystem;

public class RetractTelescopeCommand extends CommandBase {
  private TelescopeSubsystem m_telescopeSubsystem;

  public RetractTelescopeCommand(
      SubsystemContainer p_subsystemContainer, XboxController p_driverController) {
    m_telescopeSubsystem = p_subsystemContainer.getTelescopeSubsystem();
    addRequirements(m_telescopeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_telescopeSubsystem.retractTelescoping();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
