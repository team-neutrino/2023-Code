// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemContainer;

public class NEWShuffleboardSubsystem extends SubsystemBase {
  /** Creates a new NEWShuffleboardSubsystem. */
  DigitalInput limitSwitch = new DigitalInput(0);
  SubsystemContainer p_subsystemContainer;
  private NEWShuffleboardSubsystem shuffleboardSubsystem;
  private DriveTrainSubsystem m_drivetrainSubsystem;
  private ShuffleboardTab mainTab;
  
  
  public NEWShuffleboardSubsystem() {
    mainTab = Shuffleboard.getTab("Glass Application");
    m_drivetrainSubsystem = p_subsystemContainer.getDriveTrainSubsystem();
  }

  public boolean limitSwitchPressed() {
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean isPressed = limitSwitchPressed();
    double R1Position = m_drivetrainSubsystem.getR1Pos();

    mainTab.getLayout("Limit Switch Status", BuiltInLayouts.kList)
    .withSize(2, 1)
    .withPosition(0, 0)
    .add("Status", isPressed ? "Engaged" : "Not Engaged")
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withProperties(Map.of("colorWhenTrue", "green"))
    .withProperties(Map.of("colorWhenFalse", "red"));

    mainTab.getLayout("Drivetrain Data", BuiltInLayouts.kList)
    .withSize(1, 1)
    .withPosition(1, 0)
    .add("R1 Position", R1Position)
    .withWidget(BuiltInWidgets.kTextView);
  }
}
