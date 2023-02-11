package frc.robot.util;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PneumaticCompressor {
  Compressor m_Compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  public PneumaticCompressor() {}

  public void enableCompressor() {
    m_Compressor.enableDigital();
  }
}
