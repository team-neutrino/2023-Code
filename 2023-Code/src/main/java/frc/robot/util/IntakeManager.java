// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.Constants;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeManager {
  private ArmSubsystem m_armSubsystem;
  private IntakeSubsystem m_intakeSubystem;

  public IntakeManager(ArmSubsystem p_armSubsystem, IntakeSubsystem p_intakeSubsystem) {
    m_armSubsystem = p_armSubsystem;
    m_intakeSubystem = p_intakeSubsystem;
  }

  public IntakeManager(SubsystemContainer p_subsystemContainer) {
    m_armSubsystem = p_subsystemContainer.getArmSubsystem();
  }

  /**
   * If the arm is down, then we DO NOT want to put the intake down or up because it will collide
   * with the arm
   *
   * @return If it is OK to run intake-dependant commands.
   */
  public boolean managerApproved() {
    return m_armSubsystem.getAbsolutePosition() <= Constants.ArmConstants.INTAKE_RUNNABLE;
  }

  /**
   * An overriden version of the 'setIntakeDown' method in the intake subsystem that ensures that
   * the arm is not down to prevent collision.
   */
  public void setIntakeDownWithArmCheck() {
    if (managerApproved()) {
      m_intakeSubystem.setIntakeDown();
    }
  }

  /**
   * An overriden version of the 'setIntakeUp' method in the intake subsystem that ensures that the
   * arm is not down to prevent collision.
   */
  public void setIntakeUpWithArmCheck() {
    if (managerApproved()) {
      m_intakeSubystem.setIntakeUp();
    }
  }
}
