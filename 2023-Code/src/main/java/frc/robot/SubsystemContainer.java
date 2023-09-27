package frc.robot;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class SubsystemContainer {
  private DriveTrainSubsystem m_drivetrainSubsystem;
  private ScoringSubsystem m_scoringSubsystem;
  private LimelightSubsystem m_limelightSubsystem;
  private ArmSubsystem m_armSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private LEDSubsystem m_ledSubsystem;
  private TelescopeSubsystem m_telescopeSubsystem;
  public ClawSubsystem m_ClawSubsystem;

  public SubsystemContainer(
      TelescopeSubsystem p_telescopeSubsystem,
      DriveTrainSubsystem p_drivetrainSubsystem,
      ScoringSubsystem p_scoringSubsystem,
      LimelightSubsystem p_limelLimelightSubsystem,
      ArmSubsystem p_armSubsystem,
      IntakeSubsystem p_intakeSubsystem,
      LEDSubsystem p_ledSubsystem,
      ClawSubsystem p_intakeWSubsystem) {
    m_drivetrainSubsystem = p_drivetrainSubsystem;
    m_scoringSubsystem = p_scoringSubsystem;
    m_limelightSubsystem = p_limelLimelightSubsystem;
    m_armSubsystem = p_armSubsystem;
    m_intakeSubsystem = p_intakeSubsystem;
    m_ledSubsystem = p_ledSubsystem;
    m_telescopeSubsystem = p_telescopeSubsystem;
    m_ClawSubsystem = p_intakeWSubsystem;
  }

  public DriveTrainSubsystem getDriveTrainSubsystem() {
    return m_drivetrainSubsystem;
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

  public TelescopeSubsystem getTelescopeSubsystem() {
    return m_telescopeSubsystem;
  }

  public ClawSubsystem getClawSubsystem() {
    return m_ClawSubsystem;
  }
}
