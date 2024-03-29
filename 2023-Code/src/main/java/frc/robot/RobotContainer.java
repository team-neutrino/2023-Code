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
import frc.robot.commands.ArmMagicCommand;
import frc.robot.commands.ArmToAngleCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.DriveTrainDefaultCommand;
import frc.robot.commands.ExtendModeCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeGatherModeCommand;
import frc.robot.commands.IntakeHybridModeCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.IntakeSqueezeCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.ScoringCloseCommand;
import frc.robot.commands.ScoringDefaultCommand;
import frc.robot.commands.ScoringOpenCommand;
import frc.robot.commands.TelescopeCommand;
import frc.robot.commands.TelescopeDefaultCommand;
import frc.robot.commands.autonomous.ScoreHighThenBalance;
import frc.robot.commands.autonomous.ScoreMobilityThenBalance;
import frc.robot.commands.autonomous.ScoreMoveAutoGather;
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
import frc.robot.subsystems.TelescopeSubsystem;
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
  private final JoystickButton m_leftJoystickButton =
      new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value);

  // SUBSYSTEMS
  private final DriveTrainSubsystem m_drivetrainSubsystem =
      new DriveTrainSubsystem(m_leftJoystick, m_rightJoystick);
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ScoringSubsystem m_scoringSubsystem = new ScoringSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();
  private final PneumaticSubsystem m_pneumaticSubsystem = new PneumaticSubsystem();
  private final TelescopeSubsystem m_telescopeSubsystem = new TelescopeSubsystem();

  public SubsystemContainer m_subsystemContainer =
      new SubsystemContainer(
          m_telescopeSubsystem,
          m_drivetrainSubsystem,
          m_scoringSubsystem,
          m_limelightSubsystem,
          m_armSubsystem,
          m_intakeSubsystem,
          m_ledSubsystem);

  // UTIL
  private final ViennaPIDController m_armPidController =
      new ViennaPIDController(
          PIDConstants.ARM_P, PIDConstants.ARM_I, PIDConstants.ARM_D, PIDConstants.ARM_FF);
  private final IntakeManager m_intakeManager = new IntakeManager(m_subsystemContainer);
  private final DriverStationInfo m_driverStationInfo = new DriverStationInfo();

  private final ShuffleboardSubsystem m_shuffleboardSubsystem =
      new ShuffleboardSubsystem(
          m_subsystemContainer,
          m_driverStationInfo,
          m_armPidController,
          m_intakeManager,
          m_driverController);

  // DEFAULT COMMANDS
  private final ArmDefaultCommand m_armDefaultCommand =
      new ArmDefaultCommand(m_subsystemContainer, m_armPidController);
  private final DriveTrainDefaultCommand m_driveTrainDefaultCommand =
      new DriveTrainDefaultCommand(m_subsystemContainer, m_leftJoystick, m_rightJoystick);
  private final IntakeDefaultCommand m_intakeDefaultCommand =
      new IntakeDefaultCommand(m_subsystemContainer, m_intakeManager);
  private final ScoringDefaultCommand m_scoringDefaultCommand =
      new ScoringDefaultCommand(m_subsystemContainer);

  // AUTON COMMANDS
  private final AutoBalanceCommand m_autoBalanceCommand =
      new AutoBalanceCommand(m_subsystemContainer);
  private final ScoreMobilityThenBalance m_scoreMobilityThenBalance =
      new ScoreMobilityThenBalance(
          m_subsystemContainer, m_armPidController, m_intakeManager, m_driverController);
  private final ScoreThenMove m_scoreThenMove =
      new ScoreThenMove(
          m_subsystemContainer, m_armPidController, m_intakeManager, m_driverController);
  private final ScoreThenBalance m_scoreThenBalanece =
      new ScoreThenBalance(
          m_subsystemContainer, m_armPidController, m_intakeManager, m_driverController);
  private final ScoreMoveAutoGather m_scoreThenMoveThenAutoGather =
      new ScoreMoveAutoGather(
          m_subsystemContainer, m_armPidController, m_intakeManager, m_driverController);

  // INTAKE COMMANDS
  private final IntakeReverseCommand m_intakeReverseCommand =
      new IntakeReverseCommand(m_subsystemContainer, m_intakeManager);
  private final IntakeSqueezeCommand m_intakeSqueezeCommand =
      new IntakeSqueezeCommand(m_subsystemContainer);
  private final IntakeGatherModeCommand m_intakeGatherModeCommand =
      new IntakeGatherModeCommand(m_subsystemContainer, m_intakeManager, false);
  private final IntakeHybridModeCommand m_intakeHybridModeCommand =
      new IntakeHybridModeCommand(m_subsystemContainer, m_intakeManager);

  // SCORING COMMANDS
  private final ScoringCloseCommand m_scoringCloseCommand =
      new ScoringCloseCommand(m_subsystemContainer);
  private final ScoringOpenCommand m_scoringOpenCommand =
      new ScoringOpenCommand(m_subsystemContainer, m_intakeManager);

  // ARM COMMANDS
  private final ArmGatherModeCommand m_armGatherModeCommand =
      new ArmGatherModeCommand(m_subsystemContainer, m_armPidController);
  private final ArmFeederCommand m_armFeederCommand =
      new ArmFeederCommand(m_subsystemContainer, m_armPidController, m_ledSubsystem);

  private final ArmToAngleCommand m_armToForwardMid =
      new ArmToAngleCommand(
          m_subsystemContainer,
          m_armPidController,
          m_driverController,
          ArmConstants.FORWARD_MID,
          false,
          true);
  private final ArmToAngleCommand m_armToForwardDown =
      new ArmToAngleCommand(
          m_subsystemContainer,
          m_armPidController,
          m_driverController,
          ArmConstants.FORWARD_DOWN,
          false,
          false);
  private final ArmToAngleCommand m_armToBackMid =
      new ArmToAngleCommand(
          m_subsystemContainer,
          m_armPidController,
          m_driverController,
          ArmConstants.BACK_MID,
          false,
          true);
  private final ArmToAngleCommand m_armToBackDown =
      new ArmToAngleCommand(
          m_subsystemContainer,
          m_armPidController,
          m_driverController,
          ArmConstants.BACK_DOWN,
          false,
          false);

  private final ArmMagicCommand m_armMagicMid =
      new ArmMagicCommand(m_subsystemContainer, m_armPidController, false, false);
  private final ArmMagicCommand m_armMagicHigh =
      new ArmMagicCommand(m_subsystemContainer, m_armPidController, true, false);

  private final TelescopeDefaultCommand m_TelescopeDefaultCommand =
      new TelescopeDefaultCommand(m_subsystemContainer);

  // TELESCOPING ARM COMMANDS
  private final TelescopeCommand m_telescopeCommand =
      new TelescopeCommand(m_subsystemContainer, m_driverController);
  private final ExtendModeCommand m_extendModeCommand =
      new ExtendModeCommand(m_telescopeSubsystem, m_armSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_drivetrainSubsystem.setDefaultCommand(m_driveTrainDefaultCommand);
    m_scoringSubsystem.setDefaultCommand(m_scoringDefaultCommand);
    m_intakeSubsystem.setDefaultCommand(m_intakeDefaultCommand);
    m_armSubsystem.setDefaultCommand(m_armDefaultCommand);
    m_telescopeSubsystem.setDefaultCommand(m_TelescopeDefaultCommand);

    // BUTTONS
    m_buttonB.toggleOnTrue(m_armToForwardMid);
    m_buttonY.toggleOnTrue(m_armToForwardDown);
    m_buttonX.toggleOnTrue(m_armToBackMid);
    m_buttonA.toggleOnTrue(m_armToBackDown);

    m_buttonBack.whileTrue(m_scoringOpenCommand);

    // used for small adjustments of the arm
    m_rightStickButton.toggleOnTrue(
        new ArmAdjustCommand(m_subsystemContainer, m_driverController, m_armPidController));
    m_leftJoystickButton.toggleOnTrue(m_telescopeCommand);
    m_leftTrigger.whileTrue(
        new SequentialCommandGroup(m_intakeGatherModeCommand, m_armGatherModeCommand));
    m_leftBumper.whileTrue(m_armFeederCommand);

    m_rightTrigger.whileTrue(m_intakeHybridModeCommand);
    m_rightBumper.whileTrue(m_intakeReverseCommand);

    // LED Buttons
    m_rightArrow.onTrue(new LEDCommand(m_subsystemContainer, LEDColor.PURPLE, m_driverStationInfo));
    m_leftArrow.onTrue(new LEDCommand(m_subsystemContainer, LEDColor.YELLOW, m_driverStationInfo));

    // Magic Button for Arm Movement
    m_upArrow.onTrue(m_armMagicHigh);
    m_downArrow.onTrue(m_armMagicMid);
  }

  public Command getAutonomousCommand() {
    m_drivetrainSubsystem.resetOdometry();
    return new ScoreHighThenBalance(
            m_subsystemContainer, m_armPidController, m_intakeManager, m_driverController)
        .andThen(new InstantCommand(() -> m_drivetrainSubsystem.setVoltage(0, 0)));
    // return m_shuffleboardSubsystem
    //     .getAutoSelected()
    //     .andThen(new InstantCommand(() -> m_driveTrainSubsystem.setVoltage(0, 0)));
  }

  public void resetNavX() {
    m_drivetrainSubsystem.getNavX().zeroYaw();
  }
}
