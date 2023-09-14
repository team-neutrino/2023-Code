package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.NewIntakeWSubsystem;

public class NewIntakeWReverseCommand extends CommandBase {
    
    private NewIntakeWSubsystem m_newIntakeWSubsystem; 

    public NewIntakeWReverseCommand(SubsystemContainer p_subsystemContainer) {

        m_newIntakeWSubsystem = p_subsystemContainer.getnewIntakeWSubstsytem();
        addRequirements(m_newIntakeWSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    m_newIntakeWSubsystem.runMotorReverse();
  }

    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public boolean isFinished() {
      return false;
  }
}
