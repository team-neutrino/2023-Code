package frc.robot.util;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeManager {
    IntakeSubsystem m_intakeSubsystem;
    ArmSubsystem m_armSubsystem; 

    public IntakeManager(IntakeSubsystem p_intakeSubsystem, ArmSubsystem p_armSubsystem){
        m_intakeSubsystem = p_intakeSubsystem;
        m_armSubsystem = p_armSubsystem;
    }

    
}
