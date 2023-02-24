package frc.lib;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The TelemetrySubsystem class extends SubsystemBase to provide a contract for the implementation of telemetry features.
 */
public abstract class Subsystem extends SubsystemBase {
  public abstract void addToShuffleboard(ShuffleboardContainer container);

  public abstract void outputTelemetry();
}
