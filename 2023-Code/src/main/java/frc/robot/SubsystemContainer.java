package frc.robot;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;

public class SubsystemContainer {
  private DriveTrainSubsystem m_driveTrainSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private LimelightSubsystem m_limelightSubsystem;
  private ArmSubsystem m_armSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private LEDSubsystem m_ledSubsystem;
  private PneumaticSubsystem m_pneumaticSubsystem;
  private ShuffleboardSubsystem m_shuffleboardSubsystem;

  public SubsystemContainer(
      ShuffleboardSubsystem p_shuffleboardSubsystem,
      PneumaticSubsystem p_pneumaticSubsystem,
      DriveTrainSubsystem p_driveTrainSubsystem,
      ScoringSubsystem p_scoringSubsystem,
      LimelightSubsystem p_limelLimelightSubsystem,
      ArmSubsystem p_armSubsystem,
      IntakeSubsystem p_intakeSubsystem,
      LEDSubsystem p_ledSubsystem) {
    m_driveTrainSubsystem = p_driveTrainSubsystem;
    m_scoringSubsystem = p_scoringSubsystem;
    m_limelightSubsystem = p_limelLimelightSubsystem;
    m_armSubsystem = p_armSubsystem;
    m_intakeSubsystem = p_intakeSubsystem;
    m_ledSubsystem = p_ledSubsystem;
    m_pneumaticSubsystem = p_pneumaticSubsystem;
    m_shuffleboardSubsystem = p_shuffleboardSubsystem;
  }

  public DriveTrainSubsystem getDriveTrainSubsystem() {
    return m_driveTrainSubsystem;
  }

  public ScoringSubsystem getScoringSubsystem() {
    return m_scoringSubsystem;
  }

  public LimelightSubsystem getLimelightSubsystem() {
    return m_limelightSubsystem;
  }

  public ArmSubsystem getArmSubsystem() {
    return m_armSubsystem;
  }

  public IntakeSubsystem getIntakeSubsystem() {
    return m_intakeSubsystem;
  }

  public LEDSubsystem getLedSubsystem() {
    return m_ledSubsystem;
  }
}
