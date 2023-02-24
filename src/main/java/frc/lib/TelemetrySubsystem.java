package frc.lib;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The TelemetrySubsystem interface is an implementation contract for subsystems that provide telemetry information via Shuffleboard or other means.
 */
public interface TelemetrySubsystem {
  public void addToShuffleboard(ShuffleboardTab tab);

  public void outputTelemetry();
}
