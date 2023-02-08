package frc.robot.util;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeManager {
    IntakeSubsystem m_intakeSubsystem;
    ArmSubsystem m_armSubsystem; 

    public IntakeManager(IntakeSubsystem x, ArmSubsystem p_armSubsystem){
        m_intakeSubsystem = x;
        m_armSubsystem = p_armSubsystem;
    }

    public void doSomething()
    {
        //
        m_intakeSubsystem.setIntakeDown();
    }
    
}
