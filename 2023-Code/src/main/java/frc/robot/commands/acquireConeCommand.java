package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.ClawSubsystem;

public class acquireConeCommand extends CommandBase {
    
    private ClawSubsystem m_ClawSubsystem; 

    public acquireConeCommand(SubsystemContainer p_subsystemContainer, ClawSubsystem m_ClawSubsystem2) {

        m_ClawSubsystem = p_subsystemContainer.getClawSubsystem();
        addRequirements(m_ClawSubsystem);
    }

    public acquireConeCommand(SubsystemContainer m_subsystemContainer) {
    }

    @Override
    public void initialize() {
    m_ClawSubsystem.acquireCone();
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
