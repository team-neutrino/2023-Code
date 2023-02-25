package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

public class TrajectoryConfigConstants {

  // COPIED FROM ATLAS
  public static final double KV_VOLT_SECONDS_PER_METER = 2.363;
  public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.3851;
  public static final double K_TRACK_WIDTH_METERS = 0.635;
  public static final DifferentialDriveKinematics K_DRIVE_KINEMATICS =
      new DifferentialDriveKinematics(K_TRACK_WIDTH_METERS);
  public static final double K_MAX_SPEED_METERS_PER_SECOND = 3;
  public static final double K_HALF_SPEED_METERS_PER_SECOND = 1.5;
  public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2;
  public static final double K_RAMSETE_BETA = 2;
  public static final double K_RAMSETE_ZETA = 0.7;
  public static final double KP_DRIVE_VEL = .44295;
  public static final double KS_VOLTS = 0.13804;

  private static final DifferentialDriveVoltageConstraint m_autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(
              KS_VOLTS, KV_VOLT_SECONDS_PER_METER, KA_VOLT_SECONDS_SQUARED_PER_METER),
          K_DRIVE_KINEMATICS,
          10);

  public static final TrajectoryConfig m_MaxSpeedForward =
      new TrajectoryConfig(K_MAX_SPEED_METERS_PER_SECOND, K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
          .setKinematics(K_DRIVE_KINEMATICS)
          .addConstraint(m_autoVoltageConstraint);

  public static final TrajectoryConfig m_HalfSpeedForward =
      new TrajectoryConfig(K_HALF_SPEED_METERS_PER_SECOND, K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
          .setKinematics(K_DRIVE_KINEMATICS)
          .addConstraint(m_autoVoltageConstraint);
}
