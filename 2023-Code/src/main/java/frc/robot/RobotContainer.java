// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmAdjustCommand;
import frc.robot.commands.ArmDefaultCommand;
import frc.robot.commands.ArmToAngleCommand;
import frc.robot.commands.DriveTrainDefaultCommand;
import frc.robot.commands.EndGameDefaultCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.ScoringDefaultCommand;
import frc.robot.commands.ScoringOpenCommand;
import frc.robot.commands.autonomous.ForwardBackwardCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.EndGameSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.util.DriverStationInfo;

public class RobotContainer {
  // UTIL
  private final DriverStationInfo m_driverStationInfo = new DriverStationInfo();

  // SUBSYSTEMS
  private final DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem();
  private final EndGameSubsystem m_endGame = new EndGameSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ScoringSubsystem m_scoringSubsystem = new ScoringSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  private final ShuffleboardSubsystem m_shuffleboardSubsystem =
      new ShuffleboardSubsystem(
          m_driverStationInfo,
          m_driveTrain,
          m_scoringSubsystem,
          m_limelightSubsystem,
          m_armSubsystem,
          m_intakeSubsystem);

  // CONTROLLER (Replace with CommandPS4Controller or CommandJoystick if needed)
  private final XboxController m_driverController = new XboxController(OperatorConstants.XBOX);

  // JOYSTICKS
  private final Joystick m_leftJoystick = new Joystick(OperatorConstants.JOYSTICK_LEFT);
  private final Joystick m_rightJoystick = new Joystick(OperatorConstants.JOYSTICK_RIGHT);

  // BUTTONS
  private final JoystickButton m_buttonA =
      new JoystickButton(m_driverController, XboxController.Button.kA.value);
  private final JoystickButton m_buttonB =
      new JoystickButton(m_driverController, XboxController.Button.kB.value);
  private final JoystickButton m_buttonX =
      new JoystickButton(m_driverController, XboxController.Button.kX.value);
  private final JoystickButton m_buttonY =
      new JoystickButton(m_driverController, XboxController.Button.kY.value);

  // BUMPERS
  private final JoystickButton m_leftBumper =
      new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton m_rightBumper =
      new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);

  // BUTTON BABES
  private final JoystickButton m_buttonStart =
      new JoystickButton(m_driverController, XboxController.Button.kStart.value);
  private final POVButton m_upArrow = new POVButton(m_driverController, 0);
  private final POVButton m_downArrow = new POVButton(m_driverController, 180);
  private final JoystickButton m_buttonBack =
      new JoystickButton(m_driverController, XboxController.Button.kBack.value);

  // TRIGGERS
  private final Trigger m_leftTrigger =
      new Trigger(() -> m_driverController.getLeftTriggerAxis() >= .5);
  private final Trigger m_rightTrigger =
      new Trigger(() -> m_driverController.getRightTriggerAxis() >= .5);

  // COMMANDS
  private final ArmDefaultCommand m_armDefaultCommand = new ArmDefaultCommand(m_armSubsystem);

  private final DriveTrainDefaultCommand m_driveTrainDefaultCommand =
      new DriveTrainDefaultCommand(m_driveTrain, m_leftJoystick, m_rightJoystick);

  // Turn both intake motors off and set the entire thing up
  private final IntakeDefaultCommand m_IntakeDefaultCommand =
      new IntakeDefaultCommand(m_intakeSubsystem);

  // Turn both intake motors on and set the entire thing down
  private final IntakeCommand m_intakeCommand = new IntakeCommand(m_intakeSubsystem);
  private final IntakeReverseCommand m_IntakeReverseCommand =
      new IntakeReverseCommand(m_intakeSubsystem);

  // If the right joystick trigger/button is pressed, both joysticks are pushed past
  // the deadzone given in the variable constants, and the endgane pneumatics are not
  // already extended, extend them.
  private final EndGameDefaultCommand m_endGameDefaultCommand =
      new EndGameDefaultCommand(m_endGame, m_rightJoystick, m_leftJoystick);

  // Toggles scoring pneumatics to extended position
  private final ScoringDefaultCommand m_scoringDefaultCommand =
      new ScoringDefaultCommand(m_scoringSubsystem);

  // Toggle scoring pneumatics to retracted position
  private final ScoringOpenCommand m_scoringOpenCommand =
      new ScoringOpenCommand(m_scoringSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new PneumaticsSubsystem();
    new LimelightSubsystem();
    new DriverStationInfo();

    // Configure the trigger bindings
    configureBindings();
    setDefaultCommands();
  }

  // CALLED IN CONSTRUCTOR
  private void setDefaultCommands() {
    m_driveTrain.setDefaultCommand(m_driveTrainDefaultCommand);
    m_scoringSubsystem.setDefaultCommand(m_scoringDefaultCommand);
    m_intakeSubsystem.setDefaultCommand(m_IntakeDefaultCommand);
    m_endGame.setDefaultCommand(m_endGameDefaultCommand);
    m_armSubsystem.setDefaultCommand(m_armDefaultCommand);
  }

  private void configureBindings() {

    // SCORING
    m_buttonA.whileTrue(m_scoringOpenCommand);
    m_buttonB.whileTrue(m_exampleSubsystem.exampleMethodCommand());
    m_buttonY.whileTrue(new ArmToAngleCommand(m_armSubsystem, 90));

    // ARM
    m_buttonY.whileTrue(new ArmToAngleCommand(m_armSubsystem, 90));
    m_upArrow.whileTrue(new ArmAdjustCommand(m_armSubsystem, 1));
    m_downArrow.whileTrue(new ArmAdjustCommand(m_armSubsystem, -1));

    // INTAKE
    m_buttonX.whileTrue(m_intakeCommand);
    m_rightTrigger.whileTrue(m_IntakeReverseCommand);
    m_leftBumper.whileTrue(m_IntakeReverseCommand);
    m_leftTrigger.whileTrue(m_intakeCommand);
  }

  public Command getAutonomousCommand() {
    return new ForwardBackwardCommand(m_driveTrain);
  }
}
