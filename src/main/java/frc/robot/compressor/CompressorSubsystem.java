package frc.robot.compressor;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class CompressorSubsystem {
  private Compressor m_compressor =
      new Compressor(Constants.Compressor.module, PneumaticsModuleType.REVPH);

  /** Start the compressor. */
  public void run() {
    m_compressor.enableAnalog(Constants.Compressor.minPressure, Constants.Compressor.maxPressure);
  }

  /** Stop the compressor. */
  public void stop() {
    m_compressor.disable();
  }

  /**
   * Check if the compressor is running.
   *
   * @return true if the compressor is running, false otherwise.
   */
  public boolean isFull() {
    return m_compressor.getPressure() > Constants.Compressor.compressorThreshold;
  }
}
