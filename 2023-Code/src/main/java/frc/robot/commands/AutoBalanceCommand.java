package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoBalanceCommand extends CommandBase{

    private final DriveTrainSubsystem m_drivetrain;

    double ish = 0.05;
    double desiredPos = 0;
    double error;
    double voltage;

    double integral;

    double derivative;
    double previousError = 0;
    final double dt = .02;

  public AutoBalanceCommand(DriveTrainSubsystem p_drivetrain) {
    m_drivetrain = p_drivetrain;
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    error = desiredPos - m_drivetrain.getPitch();
    previousError = error;
    voltage = error * Constants.PIDConstants.BALANCE_P + 
      Constants.PIDConstants.BALANCE_I*(integral += error) * dt +
      Constants.PIDConstants.BALANCE_D*(error - previousError)/dt;
    m_drivetrain.setVoltage(voltage);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
