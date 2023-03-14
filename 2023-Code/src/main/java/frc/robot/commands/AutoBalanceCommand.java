package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.ViennaPIDController;

public class AutoBalanceCommand extends CommandBase {

  private final DriveTrainSubsystem m_drivetrain;
  private final ViennaPIDController pidController;

  private double desiredPos = 0;

  public AutoBalanceCommand(DriveTrainSubsystem p_drivetrain) {
    m_drivetrain = p_drivetrain;
    addRequirements(m_drivetrain);

    pidController = new ViennaPIDController(PIDConstants.BALANCE_P, PIDConstants.BALANCE_I, PIDConstants.BALANCE_D);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double currentRoll = m_drivetrain.getRoll();
    double output = pidController.run(currentRoll, desiredPos, DrivetrainConstants.AUTO_BALANCE_DEADZONE);
    m_drivetrain.setVoltage(-output);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
