// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ExampleCommand;

public class ShuffleboardSubsystem extends SubsystemBase {
  private ShuffleboardTab m_drivestationTab;

  private SendableChooser<Command> m_autonSelector = new SendableChooser<>();
  private GenericEntry m_driveTrainVariables[] = new GenericEntry[7];

  private DriveTrainSubsystem m_driveTrain;
  private ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public ShuffleboardSubsystem(DriveTrainSubsystem p_driveTrain) {
    m_driveTrain = p_driveTrain;
    m_drivestationTab = Shuffleboard.getTab("Driverstation Tab");

    driveStaionTab();
  }

  @Override
  public void periodic() {
    m_driveTrainVariables[0].setDouble(m_driveTrain.getR1Vel());
  }

  private void driveStaionTab() {
    m_driveTrainVariables[0] =
        m_drivestationTab
            .add("Right Motor 1 Position", 0)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();
  }

  public void setUpAutoSelector() {
    m_autonSelector.addOption("Example Auto 1", new ExampleCommand(m_exampleSubsystem));
    m_autonSelector.addOption("Example Auto 2", new ExampleCommand(m_exampleSubsystem));
    m_autonSelector.addOption("Example Auto 2", new ExampleCommand(m_exampleSubsystem));
  }

  public Command getAuto() {
    return m_autonSelector.getSelected();
  }
}
