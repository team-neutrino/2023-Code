// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.util.ViennaPIDController;

public class AutonArmGatherCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private ViennaPIDController m_pidController;
  private Timer m_timer = new Timer();
  private double time = 2;
  private boolean detected = false;
  private boolean auton;

  public AutonArmGatherCommand(
      ArmSubsystem p_armSubsystem,
      ScoringSubsystem p_scoringSubsystem,
      IntakeSubsystem p_intakeSubsystem,
      ViennaPIDController p_pidController, boolean p_auton) {
    m_armSubsystem = p_armSubsystem;
    m_scoringSubsystem = p_scoringSubsystem;
    m_intakeSubsystem = p_intakeSubsystem;
    m_pidController = p_pidController;
    auton = p_auton;

    addRequirements(m_armSubsystem, m_scoringSubsystem, m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    m_timer.start();
    m_intakeSubsystem.setIntakeDown();
  }

  @Override
  public void execute() {
    if (detected = false) {
      m_intakeSubsystem.runIntake();
      m_scoringSubsystem.openScoring();
      m_armSubsystem.smartSet(
          m_pidController.run(m_armSubsystem.getAbsolutePosition(), ArmConstants.GATHER_POSITION));
      if (m_intakeSubsystem.detectedGamePiece()) {
        detected = true;
      }
    }
    else {
      m_intakeSubsystem.stopIntake();

      m_scoringSubsystem.closeScoring();

      m_armSubsystem.smartSet(
          m_pidController.run(m_armSubsystem.getAbsolutePosition(), ArmConstants.FORWARD_MID));
            }
    // m_intakeSubsystem.setIntakeDown();
   

    //   if(m_intakeSubsystem.detectedGamePiece()){
    //     detected = true;
    //     m_intakeSubsystem.stopIntake();
    //     m_intakeSubsystem.unsqueeze();
    //   }
    //   else{
    //     m_intakeSubsystem.runIntake();
    //   }

    //   if(detected == true){
    //     m_scoringSubsystem.closeScoring();
    //     m_armSubsystem.smartSet(
    //       m_pidController.run(m_armSubsystem.getAbsolutePosition(), ArmConstants.FORWARD_MID));
    //   } else {
    //     m_armSubsystem.smartSet(
    //       m_pidController.run(m_armSubsystem.getAbsolutePosition(), ArmConstants.GATHER_POSITION));
    //     m_scoringSubsystem.openScoring();
    //   }
      


    

    // if (m_armSubsystem.getAbsolutePosition() >= ArmConstants.GATHER_POSITION) {
    //   if (m_intakeSubsystem.isIntakeDown()) {
    //     m_intakeSubsystem.unsqueeze();
    //     m_intakeSubsystem.runIntake();
    //   }
    //   m_scoringSubsystem.closeScoring();
    // } else {
    //   m_scoringSubsystem.openScoring();
    // }
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setIntakeUp();
  }

  @Override
  public boolean isFinished() {
    if (detected && Math.abs(m_armSubsystem.getAbsolutePosition() - ArmConstants.FORWARD_MID) < 1){
      return true;
    }
    return false;
  }
}
