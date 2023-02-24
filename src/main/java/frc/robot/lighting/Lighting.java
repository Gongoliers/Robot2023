package frc.robot.lighting;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleFaults;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Subsystem;
import frc.robot.Constants;
import java.awt.Color;
import java.util.Map;

public class Lighting extends Subsystem {

  private final CANdle m_candle = new CANdle(Constants.Lighting.CANDLE_ID);

  private GenericEntry m_faultIndicator;
  private SimpleWidget m_colorColorWidget;
  private GenericEntry m_colorColorView;

  private String m_currentColor;

  public Lighting() {
    m_candle.configAllSettings(config());
    off();

    addToShuffleboard(Shuffleboard.getTab("Lighting"));
  }

  /**
   * Turn the LEDs off.
   *
   * @return a command that will turn the LEDs off.
   */
  public CommandBase off() {
    return this.runOnce(() -> setLEDs(Constants.Lighting.COLOR_BLACK));
  }

  /**
   * Turn the LEDs yellow.
   *
   * @return a command that will turn the LEDs yellow.
   */
  public CommandBase yellow() {
    return this.runOnce(() -> setLEDs(Constants.Lighting.COLOR_YELLOW));
  }

  /**
   * Turn the LEDs purple.
   *
   * @return a command that will turn the LEDs purple.
   */
  public CommandBase purple() {
    return this.runOnce(() -> setLEDs(Constants.Lighting.COLOR_PURPLE));
  }

  /**
   * Turn the LEDs red.
   *
   * @return a command that will turn the LEDs red.
   */
  public CommandBase red() {
    return this.runOnce(() -> setLEDs(Constants.Lighting.COLOR_RED));
  }

  /**
   * Turn the LEDs green.
   *
   * @return a command that will turn the LEDs green.
   */
  public CommandBase green() {
    return this.runOnce(() -> setLEDs(Constants.Lighting.COLOR_GREEN));
  }

  @Override
  public void periodic() {
  }

  @Override
  public void addToShuffleboard(ShuffleboardContainer container) {
    // TODO Auto-generated method stub
    m_faultIndicator = container.add("Faults?", "None").withWidget(BuiltInWidgets.kTextView).getEntry();
    m_colorColorWidget =
        container.add("Current Color Color", false).withProperties(Map.of("colorWhenFalse", "black"));
    m_colorColorView = m_colorColorWidget.getEntry();
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub
    m_faultIndicator.setString(toString(getFaults()));
    displayColor();
  }

  /**
   * Generates a suitable "standard" configuration for the CANdle. The CANdle will always be
   * initialized with this configuration at the beginning of the match.
   *
   * @return the "standard" configuration for the CANdle.
   */
  private CANdleConfiguration config() {
    final CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.5;
    return config;
  }

  /**
   * Translates a CANdle's faults to its representation as a string.
   *
   * @param faults a filled faults buffer.
   * @return a string representing the faults.
   */
  private String toString(final CANdleFaults faults) {
    // https://api.ctr-electronics.com/phoenix/release/java/src-html/com/ctre/phoenix/motorcontrol/Faults.html#line.151
    final StringBuilder messages = new StringBuilder();

    // https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/led/CANdleFaults.html
    messages.append("APIError: ");
    messages.append(faults.APIError ? "1" : "0");
    messages.append(" BootDuringEnable: ");
    messages.append(faults.BootDuringEnable ? "1" : "0");
    messages.append(" HardwareFault: ");
    messages.append(faults.HardwareFault ? "1" : "0");
    messages.append(" ShortCircuit: ");
    messages.append(faults.ShortCircuit ? "1" : "0");
    messages.append(" SoftwareFuse: ");
    messages.append(faults.SoftwareFuse ? "1" : "0");
    messages.append(" ThermalFault: ");
    messages.append(faults.ThermalFault ? "1" : "0");
    messages.append(" V5TooHigh: ");
    messages.append(faults.V5TooHigh ? "1" : "0");
    messages.append(" V5TooLow: ");
    messages.append(faults.V5TooLow ? "1" : "0");
    messages.append(" VBatTooHigh: ");
    messages.append(faults.VBatTooHigh ? "1" : "0");
    messages.append(" VBatTooLow: ");
    messages.append(faults.VBatTooLow ? "1" : "0");

    return messages.toString();
  }

  /**
   * Handles all possible CANdle faults, including sticky faults. All faults are reported on the
   * Shuffleboard tab.
   */
  private CANdleFaults getFaults() {
    final CANdleFaults faults = new CANdleFaults();
    m_candle.getFaults(faults);

    // https://v5.docs.ctr-electronics.com/en/stable/ch17_Faults.html?highlight=sticky
    // "Sticky" faults stay asserted until they are manually cleared
    // All CANdle sticky faults are also "live" faults, so we can safely ignore the sticky faults
    m_candle.clearStickyFaults();

    return faults;
  }

  /** Displays the current color of the CANdle on the Shuffleboard tab. */
  private void displayColor() {
    // https://gist.github.com/GrantPerkins/ad829caee87ff054ca0ae156487e619d
    m_colorColorWidget.withProperties(Map.of("colorWhenTrue", m_currentColor));
    m_colorColorView.setBoolean(true);
  }

  /**
   * Sets the LEDs controlled by the CANdle to the specified color.
   *
   * @param hex a hexademical color code to set the LEDs to.
   */
  private void setLEDs(final String hex) {

    m_currentColor = hex;

    final Color color = Color.decode(hex);
    final ErrorCode error = m_candle.setLEDs(color.getRed(), color.getGreen(), color.getBlue());
    if (error != ErrorCode.OK) {
      // TODO Handle error condition
    }
  }
}
