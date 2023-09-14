package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.NewIntakeWSubsystem;

public class NewIntakeWDefaultCommand extends CommandBase {
    
    private NewIntakeWSubsystem m_newIntakeWSubsystem; 

    public NewIntakeWDefaultCommand(SubsystemContainer p_subsystemContainer, NewIntakeWSubsystem m_NewIntakeWSubsystem2) {

        m_newIntakeWSubsystem = p_subsystemContainer.getnewIntakeWSubstsytem();
        addRequirements(m_newIntakeWSubsystem);
    }

    public NewIntakeWDefaultCommand(SubsystemContainer m_subsystemContainer) {
    }

    @Override
    public void initialize() {
    m_newIntakeWSubsystem.runMotor();
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
