package frc.robot.subsystems;

import static frc.robot.Constants.*;

import java.awt.Color;

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

  private void setLEDs(String hex) {
    Color color = Color.decode(hex);
    m_candle.setLEDs(color.getRed(), color.getBlue(), color.getGreen());
  }

  public void off() {
    setLEDs(COLOR_BLACK);
  }

  public void yellow() {
    setLEDs(COLOR_YELLOW);
  }

  public void purple() {
    setLEDs(COLOR_PURPLE);
  }

  public void red() {
    setLEDs(COLOR_RED);
  }

  public void green() {
    setLEDs(COLOR_GREEN);
  }

  @Override
  public void periodic() {
    // Get faults; periodically assert that CANdle is OK
    faults();

    // TODO Add Shuffleboard feedback
  }
}
