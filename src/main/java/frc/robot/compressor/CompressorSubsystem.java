package frc.robot.compressor;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class CompressorSubsystem {
  private Compressor pnCompressor = new Compressor(Constants.Compressor.module, PneumaticsModuleType.REVPH);

  public void run() {
    // TODO: investigate enableAnalog vs enableDigital
    pnCompressor.enableAnalog(Constants.Compressor.minPressure, Constants.Compressor.maxPressure);
  }

  public void stop() {
    pnCompressor.disable();
  }

  public boolean isFull() {
    return pnCompressor.getPressure() > Constants.Compressor.compressorThreshold;
  }
}
