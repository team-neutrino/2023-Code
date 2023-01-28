// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShuffleboardSubsystem extends SubsystemBase {
  private ShuffleboardTab m_drivestationTab;
  private GenericEntry m_driveTrainVariables[] = new GenericEntry[7];

  private DriveTrainSubsystem m_driveTrain;

  public ShuffleboardSubsystem(DriveTrainSubsystem p_driveTrain) {
    m_driveTrain = p_driveTrain;
    m_drivestationTab = Shuffleboard.getTab("Driverstation Tab");

    driveStaionTab();
  }

  @Override
  public void periodic() {
    m_driveTrainVariables[0].setDouble(m_driveTrain.getR1Vel());
    m_driveTrainVariables[1].setDouble(m_driveTrain.getPitch());
  }

  private void driveStaionTab() {
    m_driveTrainVariables[0] =
        m_drivestationTab
            .add("Right Motor 1 Position", 0)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();

    m_driveTrainVariables[1] =
        m_drivestationTab.add("NavX Pitch", 0).withPosition(1, 1).withSize(1, 1).getEntry();
  }
}
