// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrainDefaultCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // SUBSYSTEMS
  private final DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final PneumaticsSubsystem m_pneumaticsSubsystem = new PneumaticsSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // CONTROLLERS
  private final XboxController m_driverController = new XboxController(4);
  private final Joystick m_leftJoystick = new Joystick(OperatorConstants.JoyStickPortL);
  private final Joystick m_rightJoystick = new Joystick(OperatorConstants.JoyStickPortR);

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

  // COMMANDS
  private final DriveTrainDefaultCommand m_driveTrainDefaultCommand =
      new DriveTrainDefaultCommand(m_driveTrain, m_leftJoystick, m_rightJoystick);
  private final IntakeDefaultCommand m_IntakeDefaultCommand =
      new IntakeDefaultCommand(m_IntakeSubsystem);
  private final IntakeCommand m_IntakeCommand = new IntakeCommand(m_IntakeSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    m_IntakeSubsystem.setDefaultCommand(m_IntakeDefaultCommand);

    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Pneumatics Testboard
    m_buttonA.toggleOnTrue(
        Commands.startEnd(
            m_pneumaticsSubsystem::setSolenoidOn,
            m_pneumaticsSubsystem::setSolenoidOff,
            m_pneumaticsSubsystem));

    m_leftBumper.whileTrue(m_IntakeCommand);
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_buttonB.whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
