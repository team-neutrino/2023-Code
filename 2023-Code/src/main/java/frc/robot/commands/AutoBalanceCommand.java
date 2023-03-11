package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.ViennaPIDController;

public class AutoBalanceCommand extends CommandBase {

  private final DriveTrainSubsystem m_drivetrainSubsystem;

  private double desiredPos = 0;
  private double voltage;

  private ViennaPIDController PIDController;

  public AutoBalanceCommand(DriveTrainSubsystem p_drivetrainSubsystem) {
    m_drivetrainSubsystem = p_drivetrainSubsystem;
    PIDController = new ViennaPIDController(PIDConstants.BALANCE_P);
    addRequirements(m_drivetrainSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    voltage = PIDController.run(desiredPos, m_drivetrainSubsystem.getRoll(), DrivetrainConstants.AUTO_BALANCE_DEADZONE);
    m_drivetrainSubsystem.setVoltage(-voltage);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
