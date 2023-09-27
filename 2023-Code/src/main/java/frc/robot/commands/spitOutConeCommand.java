package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.ClawSubsystem;

public class spitOutConeCommand extends CommandBase {
    
    private ClawSubsystem m_ClawSubsystem; 

    public spitOutConeCommand(SubsystemContainer p_subsystemContainer) {

        m_ClawSubsystem = p_subsystemContainer.getClawSubsystem();
        addRequirements(m_ClawSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    m_ClawSubsystem.spitOutCone();
  }

    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public boolean isFinished() {
      return false;
  }
}
