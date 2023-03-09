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
import frc.robot.commands.autonomous.manualGeneration.BlueScoreThenMoveThenAutoGather;
import frc.robot.commands.autonomous.manualGeneration.JustScore;
import frc.robot.commands.autonomous.manualGeneration.RedScoreThenMoveThenAutoGather;
import frc.robot.commands.autonomous.manualGeneration.ScoreMobilityThenBalance;
import frc.robot.commands.autonomous.manualGeneration.ScoreThenBalance;
import frc.robot.commands.autonomous.manualGeneration.ScoreThenMove;
import frc.robot.commands.autonomous.manualGeneration.ScoreThenMoveThenAutoGather;
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

public class SubsystemContainer {
  private DriveTrainSubsystem m_driveTrainSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private LimelightSubsystem m_limelightSubsystem;
  private ArmSubsystem m_armSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private LEDSubsystem m_ledSubsystem;

  public SubsystemContainer(DriveTrainSubsystem p_driveTrainSubsystem, ScoringSubsystem p_scoringSubsystem, LimelightSubsystem p_limelLimelightSubsystem,ArmSubsystem p_armSubsystem, IntakeSubsystem p_inIntakeSubsystem, LEDSubsystem p_ledSubsystem ) {
    m_driveTrainSubsystem = p_driveTrainSubsystem;
    m_scoringSubsystem = p_scoringSubsystem;
    m_limelightSubsystem = p_limelLimelightSubsystem;
    m_armSubsystem = p_armSubsystem;
    m_intakeSubsystem = p_inIntakeSubsystem;
    m_ledSubsystem = p_ledSubsystem;
    

  }

  public DriveTrainSubsystem getDriveTrainSubsystem() {
    return m_driveTrainSubsystem;
  }

  public ScoringSubsystem getScoringSubsystem(){
    return m_scoringSubsystem;

  }

  public LimelightSubsystem getLimelightSubsystem(){
    return m_limelightSubsystem;
  }

  public ArmSubsystem getArmSubsystem(){
    return m_armSubsystem;
  }

  public IntakeSubsystem getIntakeSubsystem(){
    return m_intakeSubsystem;
  }

  public LEDSubsystem getLedSubsystem(){
    return m_ledSubsystem;
  }




}
    