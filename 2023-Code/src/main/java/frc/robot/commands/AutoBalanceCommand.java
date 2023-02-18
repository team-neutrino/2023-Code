package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoBalanceCommand extends CommandBase {

  private final DriveTrainSubsystem m_drivetrain;

  private double desiredPos = 0;
  private double error;
  private double previousError = 0;

  private double voltage;
  private double integral = 0;

  public AutoBalanceCommand(DriveTrainSubsystem p_drivetrain) {
    m_drivetrain = p_drivetrain;
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    previousError = error;
    error = desiredPos - m_drivetrain.getPitch();
    if (Math.abs(error) <= DrivetrainConstants.AUTO_BALANCE_DEADZONE) {
      error = 0;
    }
    voltage =
        error * PIDConstants.BALANCE_P
            + PIDConstants.BALANCE_I * (integral += (error * PIDConstants.dt))
            + PIDConstants.BALANCE_D * (error - previousError) / PIDConstants.dt;
    m_drivetrain.setVoltage(voltage, voltage);
    previousError = error;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
