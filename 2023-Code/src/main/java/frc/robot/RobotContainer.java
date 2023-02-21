// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.commands.ArmAdjustCommand;
import frc.robot.commands.ArmDefaultCommand;
import frc.robot.commands.ArmToAngleCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoProcessCommand;
import frc.robot.commands.DriveTrainDefaultCommand;
import frc.robot.commands.DriveTrainDriveFowardCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.ScoringCloseCommand;
import frc.robot.commands.ScoringDefaultCommand;
import frc.robot.commands.autonomous.TestAuton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ColorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.util.DriverStationInfo;
import frc.robot.util.EnumConstants.LEDColor;
import frc.robot.util.IntakeManager;
import frc.robot.util.ViennaPIDController;

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

  private final JoystickButton m_buttonR6 =
      new JoystickButton(m_rightJoystick, 8);

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

  // SUBSYSTEMS
  private final DriveTrainSubsystem m_driveTrainSubsystem =
      new DriveTrainSubsystem(m_leftJoystick, m_rightJoystick);
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ScoringSubsystem m_scoringSubsystem = new ScoringSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();
  private final PneumaticSubsystem m_PneumaticSubsystem = new PneumaticSubsystem();
  private final ColorSubsystem m_colorSubsystem = new ColorSubsystem();

  // UTIL
  private final DriverStationInfo m_driverStationInfo = new DriverStationInfo();
  private final ViennaPIDController m_armPidController =
      new ViennaPIDController(PIDConstants.ARM_P, PIDConstants.ARM_I, PIDConstants.ARM_D);
  private IntakeManager m_intakeManager = new IntakeManager(m_armSubsystem, m_intakeSubsystem);

  // SHUFFLEBOARD
  private final ShuffleboardSubsystem m_shuffleboardSubsystem =
      new ShuffleboardSubsystem(
          m_driverStationInfo,
          m_driveTrainSubsystem,
          m_scoringSubsystem,
          m_limelightSubsystem,
          m_armSubsystem,
          m_intakeSubsystem,
          m_colorSubsystem,
          m_ledSubsystem);

  // COMMANDS
  private final ArmDefaultCommand m_armDefaultCommand = new ArmDefaultCommand(m_armSubsystem);
  private final DriveTrainDefaultCommand m_driveTrainDefaultCommand =
      new DriveTrainDefaultCommand(m_driveTrainSubsystem, m_leftJoystick, m_rightJoystick);
  private final IntakeDefaultCommand m_intakeDefaultCommand =
      new IntakeDefaultCommand(m_intakeSubsystem, m_intakeManager);
  private final ScoringDefaultCommand m_scoringDefaultCommand =
      new ScoringDefaultCommand(m_scoringSubsystem);

  private final AutoBalanceCommand m_autoBalanceCommand =
      new AutoBalanceCommand(m_driveTrainSubsystem);
  private final AutoProcessCommand m_autoProcessCommand =
      new AutoProcessCommand(m_intakeSubsystem, m_armSubsystem, m_scoringSubsystem);
  private final DriveTrainDriveFowardCommand m_DriveTrainDriveFowardCommand =
      new DriveTrainDriveFowardCommand(m_driveTrainSubsystem, m_limelightSubsystem, 0);
  private final IntakeCommand m_intakeCommand =
      new IntakeCommand(m_intakeSubsystem, m_intakeManager);
  private final IntakeReverseCommand m_intakeReverseCommand =
      new IntakeReverseCommand(m_intakeSubsystem, m_intakeManager);
  private final ScoringCloseCommand m_scoringCloseCommand =
      new ScoringCloseCommand(m_scoringSubsystem);
  private final LEDCommand m_LedDefaultCommand =
      new LEDCommand(m_ledSubsystem, LEDColor.ORANGE, m_scoringSubsystem, m_driverStationInfo);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driveTrainSubsystem.setDefaultCommand(m_driveTrainDefaultCommand);
    m_scoringSubsystem.setDefaultCommand(m_scoringDefaultCommand);
    m_intakeSubsystem.setDefaultCommand(m_intakeDefaultCommand);
    m_armSubsystem.setDefaultCommand(m_armDefaultCommand);

    // BUTTONS

    m_rightBumper.toggleOnTrue(m_DriveTrainDriveFowardCommand);

    // Put forward
    // Put the arm to one of three specified target angles
    m_buttonB.whileTrue(
        new ArmToAngleCommand(m_armSubsystem, m_armPidController, ArmConstants.FORWARD_MID));
    m_buttonY.whileTrue(
        new ArmToAngleCommand(m_armSubsystem, m_armPidController, ArmConstants.FORWARD_DOWN));
    m_buttonX.whileTrue(
        new ArmToAngleCommand(m_armSubsystem, m_armPidController, ArmConstants.BACK_MID));
    m_buttonA.whileTrue(
        new ArmToAngleCommand(m_armSubsystem, m_armPidController, ArmConstants.BACK_DOWN));

    // used for small adjustments of the arm
    m_upArrow.whileTrue(new ArmAdjustCommand(m_armSubsystem, .2));
    m_downArrow.whileTrue(new ArmAdjustCommand(m_armSubsystem, -.2));
    m_leftBumper.whileTrue(m_intakeReverseCommand);
    m_leftTrigger.whileTrue(m_intakeCommand);
    m_rightBumper.whileTrue(new ScoringCloseCommand(m_scoringSubsystem));

    // LED Buttons
    m_buttonStart.onTrue(
        new LEDCommand(m_ledSubsystem, LEDColor.PURPLE, m_scoringSubsystem, m_driverStationInfo));
    m_buttonBack.onTrue(
        new LEDCommand(m_ledSubsystem, LEDColor.YELLOW, m_scoringSubsystem, m_driverStationInfo));
  }

  public Command getAutonomousCommand() {
    return new TestAuton(m_driveTrainSubsystem);
  }
}
