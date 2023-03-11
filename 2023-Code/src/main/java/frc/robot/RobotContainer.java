// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.commands.ArmAdjustCommand;
import frc.robot.commands.ArmDefaultCommand;
import frc.robot.commands.ArmFeederCommand;
import frc.robot.commands.ArmGatherModeCommand;
import frc.robot.commands.ArmToAngleCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.DriveTrainDefaultCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeGatherModeCommand;
import frc.robot.commands.IntakeHybridModeCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.IntakeSqueezeCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.ScoringCloseCommand;
import frc.robot.commands.ScoringDefaultCommand;
import frc.robot.commands.ScoringOpenCommand;
import frc.robot.commands.autonomous.ScoreMoveAutoGather;
import frc.robot.commands.autonomous.JustScore;
import frc.robot.commands.autonomous.ScoreMobilityThenBalance;
import frc.robot.commands.autonomous.ScoreThenBalance;
import frc.robot.commands.autonomous.ScoreThenMove;
import frc.robot.subsystems.ArmSubsystem;
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

  // SUBSYSTEMS
  private final DriveTrainSubsystem m_driveTrainSubsystem =
      new DriveTrainSubsystem(m_leftJoystick, m_rightJoystick);
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ScoringSubsystem m_scoringSubsystem = new ScoringSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();
  private final PneumaticSubsystem m_PneumaticSubsystem = new PneumaticSubsystem();

  // UTIL
  private final DriverStationInfo m_driverStationInfo = new DriverStationInfo();
  private final ViennaPIDController m_armPidController =
      new ViennaPIDController(PIDConstants.ARM_P, PIDConstants.ARM_I, PIDConstants.ARM_D);
  private final ViennaPIDController m_armPidControllerAdjust =
      new ViennaPIDController(PIDConstants.ARM_P_ADJUST, PIDConstants.ARM_I, PIDConstants.ARM_D);
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
          m_ledSubsystem);

  // COMMANDS
  private final ArmDefaultCommand m_armDefaultCommand =
      new ArmDefaultCommand(m_armSubsystem, m_armPidController);
  private final DriveTrainDefaultCommand m_driveTrainDefaultCommand =
      new DriveTrainDefaultCommand(m_driveTrainSubsystem, m_leftJoystick, m_rightJoystick);
  private final IntakeDefaultCommand m_intakeDefaultCommand =
      new IntakeDefaultCommand(m_intakeSubsystem, m_intakeManager);
  private final ScoringDefaultCommand m_scoringDefaultCommand =
      new ScoringDefaultCommand(m_scoringSubsystem);

  private final AutoBalanceCommand m_autoBalanceCommand =
      new AutoBalanceCommand(m_driveTrainSubsystem);

  private final ScoreMobilityThenBalance m_scoreMobilityThenBalance =
      new ScoreMobilityThenBalance(
          m_driveTrainSubsystem,
          m_armPidController,
          m_armSubsystem,
          m_scoringSubsystem,
          m_intakeSubsystem,
          m_intakeManager,
          m_ledSubsystem);
  private final ScoreThenMove m_scoreThenMove =
      new ScoreThenMove(
          m_driveTrainSubsystem,
          m_armPidController,
          m_armSubsystem,
          m_scoringSubsystem,
          m_intakeSubsystem,
          m_intakeManager,
          m_ledSubsystem);
  private final ScoreThenBalance m_scoreThenBalanece =
      new ScoreThenBalance(
          m_driveTrainSubsystem,
          m_armPidController,
          m_armSubsystem,
          m_scoringSubsystem,
          m_intakeSubsystem,
          m_intakeManager,
          m_ledSubsystem);
  private final JustScore m_justScore =
      new JustScore(
          m_driveTrainSubsystem,
          m_armPidController,
          m_armSubsystem,
          m_scoringSubsystem,
          m_intakeSubsystem,
          m_intakeManager,
          m_ledSubsystem);

  private final ScoreMoveAutoGather m_scoreThenMoveThenAutoGather =
      new ScoreMoveAutoGather(
          m_driveTrainSubsystem,
          m_armPidController,
          m_armSubsystem,
          m_scoringSubsystem,
          m_intakeSubsystem,
          m_intakeManager,
          m_ledSubsystem);

  private final IntakeCommand m_intakeCommand =
      new IntakeCommand(m_intakeSubsystem, m_intakeManager);
  private final IntakeReverseCommand m_intakeReverseCommand =
      new IntakeReverseCommand(m_intakeSubsystem, m_intakeManager);
  private final IntakeSqueezeCommand m_intakeSqueezeCommand =
      new IntakeSqueezeCommand(m_intakeSubsystem);
  private final IntakeGatherModeCommand m_intakeGatherModeCommand =
      new IntakeGatherModeCommand(m_intakeSubsystem, m_intakeManager, false);
  private final ArmGatherModeCommand m_armGatherModeCommand =
      new ArmGatherModeCommand(
          m_armSubsystem, m_scoringSubsystem, m_intakeSubsystem, m_armPidController);
  private final ArmFeederCommand m_armFeederCommand =
      new ArmFeederCommand(m_armSubsystem, m_scoringSubsystem, m_armPidController);
  private final IntakeHybridModeCommand m_intakeHybridModeCommand =
      new IntakeHybridModeCommand(m_intakeSubsystem, m_intakeManager);
  private final ScoringCloseCommand m_scoringCloseCommand =
      new ScoringCloseCommand(m_scoringSubsystem);
  private final ScoringOpenCommand m_scoringOpenCommand =
      new ScoringOpenCommand(m_scoringSubsystem, m_intakeManager);

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

    // Put the arm to one of three specified target angles
    m_buttonB.toggleOnTrue(
        new ArmToAngleCommand(
            m_armSubsystem,
            m_armPidController,
            ArmConstants.FORWARD_MID,
            false,
            false,
            m_ledSubsystem));
    m_buttonY.toggleOnTrue(
        new ArmToAngleCommand(
            m_armSubsystem,
            m_armPidController,
            ArmConstants.FORWARD_DOWN,
            false,
            false,
            m_ledSubsystem));
    m_buttonX.toggleOnTrue(
        new ArmToAngleCommand(
            m_armSubsystem,
            m_armPidController,
            ArmConstants.BACK_MID,
            false,
            true,
            m_ledSubsystem));
    m_buttonA.toggleOnTrue(
        new ArmToAngleCommand(
            m_armSubsystem,
            m_armPidController,
            ArmConstants.BACK_DOWN,
            false,
            false,
            m_ledSubsystem));

    // used for small adjustments of the arm
    m_rightStickButton.toggleOnTrue(
        new ArmAdjustCommand(m_armSubsystem, m_driverController, m_armPidControllerAdjust));

    m_leftTrigger.whileTrue(
        new SequentialCommandGroup(m_intakeGatherModeCommand, m_armGatherModeCommand));
    m_leftBumper.whileTrue(m_armFeederCommand);

    m_rightTrigger.whileTrue(m_intakeHybridModeCommand);
    m_rightBumper.whileTrue(m_intakeReverseCommand);

    m_buttonStart.whileTrue(m_intakeSqueezeCommand);
    m_buttonBack.whileTrue(m_scoringOpenCommand);

    // LED Buttons
    m_rightArrow.onTrue(
        new LEDCommand(m_ledSubsystem, LEDColor.PURPLE, m_scoringSubsystem, m_driverStationInfo));
    m_leftArrow.onTrue(
        new LEDCommand(m_ledSubsystem, LEDColor.YELLOW, m_scoringSubsystem, m_driverStationInfo));
  }

  public Command getAutonomousCommand() {
    m_driveTrainSubsystem.resetOdometry();

    return m_scoreThenMoveThenAutoGather.andThen(
        new InstantCommand(() -> m_driveTrainSubsystem.setVoltage(0, 0)));
  }
}
