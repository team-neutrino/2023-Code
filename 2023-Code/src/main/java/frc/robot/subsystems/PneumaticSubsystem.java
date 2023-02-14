package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase {

    private Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    public PneumaticSubsystem() {}

    public void enableCompressor() {
        m_compressor.enableDigital();
    }

    @Override
    public void periodic() {
        enableCompressor();
    }
}
