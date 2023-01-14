package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleFaults;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightingSubsystem extends SubsystemBase {

  private final CANdle m_candle = new CANdle(CANDLE_ID);
  private final CANdleFaults m_faults = new CANdleFaults();

  private final ShuffleboardTab m_tab;

  public LightingSubsystem() {
    m_tab = Shuffleboard.getTab("Lighting");
    m_candle.configAllSettings(config());
    off();
  }

  private CANdleConfiguration config() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.5;
    return config;
  }

  private CANdleFaults faults() {
    m_candle.getFaults(m_faults);
    // TODO Implement fault-checking
    return m_faults;
  }

  public void off() {
    m_candle.setLEDs(0, 0, 0);
  }

  public void yellow() {
    m_candle.setLEDs(93, 87, 6);
  }

  public void purple() {
    m_candle.setLEDs(66, 9, 70);
  }

  @Override
  public void periodic() {
    // Get faults; periodically assert that CANdle is OK
    faults();

    // TODO Add Shuffleboard feedback
  }
}
