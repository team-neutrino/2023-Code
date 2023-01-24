// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
<<<<<<< Updated upstream
=======
import frc.robot.commands.ArmDefaultCommand;
import frc.robot.commands.ArmAdjustCommand;
import frc.robot.commands.ArmToAngleCommand;
>>>>>>> Stashed changes
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrainDefaultCommand;
import frc.robot.commands.EndGameDefaultCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.ScoringDefaultCommand;
import frc.robot.commands.ScoringOpenCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.EndGameSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
<<<<<<< Updated upstream
=======
// import frc.robot.util.DriverStationInfo;
>>>>>>> Stashed changes

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
<<<<<<< Updated upstream
  // The robot's subsystems and commands are defined here...
=======

  // UTIL
//   private final DriverStationInfo m_driverStationInfo = new DriverStationInfo();
>>>>>>> Stashed changes

  // SUBSYSTEMS
  private final DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem();
  private final EndGameSubsystem m_endGame = new EndGameSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ScoringSubsystem m_scoringSubsystem = new ScoringSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // CONTROLLERS
  private final XboxController m_driverController = new XboxController(OperatorConstants.XBOX);
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
  private final JoystickButton m_leftBumper =
      new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton m_rightBumper =
      new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
  private final JoystickButton m_buttonStart =
      new JoystickButton(m_driverController, XboxController.Button.kStart.value);
  private final JoystickButton m_buttonBack =
      new JoystickButton(m_driverController, XboxController.Button.kBack.value);
  private final Trigger m_leftTrigger =
      new Trigger(() -> m_driverController.getLeftTriggerAxis() >= .5);
  private final Trigger m_rightTrigger =
      new Trigger(() -> m_driverController.getRightTriggerAxis() >= .5);

  // COMMANDS
  private final DriveTrainDefaultCommand m_driveTrainDefaultCommand =
      new DriveTrainDefaultCommand(m_driveTrain, m_leftJoystick, m_rightJoystick);

  // turn both intake motors off and set the entire thing up
  private final IntakeDefaultCommand m_IntakeDefaultCommand =
      new IntakeDefaultCommand(m_intakeSubsystem);

  // turn both intake motors on and set the entire thing down
  private final IntakeCommand m_intakeCommand = new IntakeCommand(m_intakeSubsystem);

  private final IntakeReverseCommand m_IntakeReverseCommand =
      new IntakeReverseCommand(m_intakeSubsystem);

  // if the right joystick trigger/button is pressed, both joysticks are pushed past
  // the deadzone given in the variable constants, and the endgane pneumatics are not
  // already extended, extend them.
  private final EndGameDefaultCommand m_endGameDefaultCommand =
      new EndGameDefaultCommand(m_endGame, m_rightJoystick, m_leftJoystick);

  // toggles scoring pneumatics to extended position
  private final ScoringDefaultCommand m_scoringDefaultCommand =
      new ScoringDefaultCommand(m_scoringSubsystem);

  // toggle scoring pneumatics to retracted position
  private final ScoringOpenCommand m_scoringOpenCommand =
      new ScoringOpenCommand(m_scoringSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new PneumaticsSubsystem();
    new LimelightSubsystem();
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_driveTrain.setDefaultCommand(m_driveTrainDefaultCommand);
    m_scoringSubsystem.setDefaultCommand(m_scoringDefaultCommand);
    m_intakeSubsystem.setDefaultCommand(m_IntakeDefaultCommand);
    m_endGame.setDefaultCommand(m_endGameDefaultCommand);

    m_buttonA.whileTrue(m_scoringOpenCommand);
    m_rightTrigger.whileTrue(m_IntakeReverseCommand);
    m_leftTrigger.whileTrue(m_intakeCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will /be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
