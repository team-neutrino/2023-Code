// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.MotorSubsystem;

public class RobotContainer {
  // CONTROLLERS
  private final XboxController m_driverController = new XboxController(OperatorConstants.XBOX);
  private final Joystick m_leftJoystick = new Joystick(OperatorConstants.JOYSTICK_LEFT);
  private final Joystick m_rightJoystick = new Joystick(OperatorConstants.JOYSTICK_RIGHT);

  // XBOX BUTTON BABES
  private final JoystickButton m_buttonA =
      new JoystickButton(m_driverController, XboxController.Button.kA.value);
  private final JoystickButton m_buttonB =
      new JoystickButton(m_driverController, XboxController.Button.kB.value);
  private final JoystickButton m_buttonX =
      new JoystickButton(m_driverController, XboxController.Button.kX.value);
  private final JoystickButton m_buttonY =
      new JoystickButton(m_driverController, XboxController.Button.kY.value);

  private final Trigger m_leftBumper =
      new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton m_rightBumper =
      new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);

  private final Trigger m_leftTrigger =
      new Trigger(() -> m_driverController.getLeftTriggerAxis() >= .5);
  private final Trigger m_rightTrigger =
      new Trigger(() -> m_driverController.getRightTriggerAxis() >= .5);

  private final JoystickButton m_buttonStart =
      new JoystickButton(m_driverController, XboxController.Button.kStart.value);
  private final JoystickButton m_buttonBack =
      new JoystickButton(m_driverController, XboxController.Button.kBack.value);

  private final POVButton m_upArrow = new POVButton(m_driverController, 0);
  private final POVButton m_downArrow = new POVButton(m_driverController, 180);
  private final POVButton m_leftArrow = new POVButton(m_driverController, 270);
  private final POVButton m_rightArrow = new POVButton(m_driverController, 90);

  private final JoystickButton m_rightStickButton =
      new JoystickButton(m_driverController, XboxController.Button.kRightStick.value);
  private final JoystickButton m_leftJoystickButton =
      new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value);

  // SUBSYSTEMS
  private final MotorSubsystem m_drivetrainSubsystem = new MotorSubsystem(m_rightJoystick);

  // UTIL

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  // BUTTONS

  // used for small adjustments of the arm

  public void resetNavX() {}
}
