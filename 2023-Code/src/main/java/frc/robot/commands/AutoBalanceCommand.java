package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.SubsystemContainer;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.ViennaPIDController;

public class AutoBalanceCommand extends CommandBase {

  private DriveTrainSubsystem m_drivetrainSubsystem;
  private ViennaPIDController m_pidController;

  private double desiredPos = 0;
  private double voltage;

  public AutoBalanceCommand(
      SubsystemContainer p_subsystemContainer, ViennaPIDController p_pidController) {
    m_drivetrainSubsystem = p_subsystemContainer.getDriveTrainSubsystem();
    m_pidController = p_pidController;
    addRequirements(m_drivetrainSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    voltage =
        m_pidController.run(
            desiredPos, m_drivetrainSubsystem.getRoll(), DrivetrainConstants.AUTO_BALANCE_DEADZONE);
    m_drivetrainSubsystem.setVoltage(-voltage);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
