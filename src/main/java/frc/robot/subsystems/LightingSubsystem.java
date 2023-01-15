package frc.robot.subsystems;

import static frc.robot.Constants.CANDLE_ID;
import static frc.robot.Constants.COLOR_BLACK;
import static frc.robot.Constants.COLOR_GREEN;
import static frc.robot.Constants.COLOR_PURPLE;
import static frc.robot.Constants.COLOR_RED;
import static frc.robot.Constants.COLOR_YELLOW;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleFaults;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.awt.Color;
import java.util.ArrayList;
import java.util.Map;

public class LightingSubsystem extends SubsystemBase {

  private final CANdle m_candle = new CANdle(CANDLE_ID);

  private final ShuffleboardTab m_tab;
  private final GenericEntry m_faultIndicator;
  private final GenericEntry m_colorStringView;
  private final SimpleWidget m_colorColorWidget;
  private final GenericEntry m_colorColorView;

  private String m_currentColor;

  public LightingSubsystem() {
    m_tab = Shuffleboard.getTab("Lighting");

    m_faultIndicator = m_tab.add("Faults?", "None").withWidget(BuiltInWidgets.kTextView).getEntry();
    m_colorStringView =
        m_tab.add("Current Color", "null").withWidget(BuiltInWidgets.kTextView).getEntry();
    m_colorColorWidget =
        m_tab.add("Current Color", false).withProperties(Map.of("colorWhenFalse", "black"));
    m_colorColorView = m_colorColorWidget.getEntry();

    m_candle.configAllSettings(config());
    off();
  }

  private CANdleConfiguration config() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.5;
    return config;
  }

  private String asMessage(CANdleFaults faults) {
    ArrayList<String> messages = new ArrayList<String>();

    // https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/led/CANdleFaults.html
    messages.add(faults.APIError ? "APIError" : "");
    messages.add(faults.BootDuringEnable ? "BootDuringEnable" : "");
    messages.add(faults.HardwareFault ? "HardwareFault" : "");
    messages.add(faults.ShortCircuit ? "ShortCircuit" : "");
    messages.add(faults.SoftwareFuse ? "SoftwareFuse" : "");
    messages.add(faults.ThermalFault ? "ThermalFault" : "");
    messages.add(faults.V5TooHigh ? "V5TooHigh" : "");
    messages.add(faults.V5TooLow ? "V5TooLow" : "");
    messages.add(faults.VBatTooHigh ? "VBatTooHigh" : "");
    messages.add(faults.VBatTooLow ? "VBatTooLow" : "");

    // Remove all "empty faults" (faults which do not occur)
    messages.removeIf(item -> "".equals(item));

    if (messages.size() == 1) {
      // If there is only one fault, return the first fault
      return messages.get(0);
    }

    // If there are multiple faults, return the combined faults
    return String.join(", ", messages);
  }

  private void handleFaults() {
    final CANdleFaults faults = new CANdleFaults();
    m_candle.getFaults(faults);

    // https://v5.docs.ctr-electronics.com/en/stable/ch17_Faults.html?highlight=sticky
    // “Sticky” faults stay asserted until they are manually cleared
    // All CANdle sticky faults are also "live" faults, so we can safely ignore the sticky faults
    m_candle.clearStickyFaults();

    if (faults.hasAnyFault() == false) {
      m_faultIndicator.setString("None");
    } else {
      m_faultIndicator.setString(asMessage(faults));
    }
  }

  private void displayColor() {
    // Set the string value to the current color
    m_colorStringView.setString(m_currentColor);

    // Use a Boolean widget to display the color of the LEDs
    // https://gist.github.com/GrantPerkins/ad829caee87ff054ca0ae156487e619d
    m_colorColorWidget.withProperties(Map.of("colorWhenTrue", m_currentColor));
    m_colorColorView.setBoolean(true);
  }

  private void setLEDs(String hex) {

    m_currentColor = hex;

    Color color = Color.decode(hex);
    ErrorCode error = m_candle.setLEDs(color.getRed(), color.getGreen(), color.getBlue());
    if (error != ErrorCode.OK) {
      // TODO Handle error condition
    }
  }

  public CommandBase off() {
    return this.runOnce(() -> setLEDs(COLOR_BLACK));
  }

  public CommandBase yellow() {
    return this.runOnce(() -> setLEDs(COLOR_YELLOW));
  }

  public CommandBase purple() {
    return this.runOnce(() -> setLEDs(COLOR_PURPLE));
  }

  public CommandBase red() {
    return this.runOnce(() -> setLEDs(COLOR_RED));
  }

  public CommandBase green() {
    return this.runOnce(() -> setLEDs(COLOR_GREEN));
  }

  @Override
  public void periodic() {
    handleFaults();
    displayColor();
  }
}
