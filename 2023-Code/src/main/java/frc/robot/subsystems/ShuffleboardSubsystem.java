// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;

public class ShuffleboardSubsystem extends SubsystemBase {
  private ShuffleboardTab m_drivestationTab;
  private GenericEntry m_driveTrainVariables[] = new GenericEntry[7];

  private SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  private ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private DriveTrainSubsystem m_driveTrain;

  public ShuffleboardSubsystem(DriveTrainSubsystem p_driveTrain, ExampleSubsystem p_exampleSubsystem) {
    m_driveTrain = p_driveTrain;
    m_exampleSubsystem = p_exampleSubsystem;
    m_drivestationTab = Shuffleboard.getTab("Driverstation Tab");

    driveStaionTab();
    setUpSelector();
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

    m_drivestationTab.add("Autonomous Chooser", m_autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1);
  }

  public void setUpSelector(){
    m_autoChooser.addOption("Auto 1", new ExampleCommand(m_exampleSubsystem));
    m_autoChooser.addOption("Auto 2", new ExampleCommand(m_exampleSubsystem));
  }

  public Command getAutoSelected(){
    return m_autoChooser.getSelected();
  }

  private void addAutonSelector(SendableChooser<Command> p_autoSelector)
  {
    m_drivestationTab.add("Autonomous Chooser", p_autoSelector)
    .withWidget(BuiltInWidgets.kComboBoxChooser)
    .withPosition(0, 0)
    .withSize(2, 1);
  }
}
