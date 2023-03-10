package frc.robot.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.thegongoliers.math.GMath;
import com.thegongoliers.output.interfaces.Lockable;
import com.thegongoliers.output.interfaces.Stoppable;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.TelemetrySubsystem;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.Arm.Extension;
import frc.robot.Robot;
import java.util.Map;

public class ExtensionController extends SubsystemBase
    implements Lockable, Stoppable, TelemetrySubsystem {

  private final WPI_TalonFX m_motor;
  private final Solenoid m_brake;

  public ExtensionController() {
    m_motor = new WPI_TalonFX(Extension.MOTOR_ID, Constants.Arm.CANBUS_NAME);
    configExtensionMotor();

    m_brake =
        new Solenoid(
            Constants.PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, Extension.BRAKE_CHANNEL);

    lock();

    // Assumes that the arm begins in the stowed state
    setLength(Constants.Arm.States.STOWED.getLength());
  }

  /**
   * Manually drive the motor at the desired speed.
   *
   * @param percent the speed to drive the motor at.
   */
  public void drive(double percent) {
    m_motor.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Sets the motor voltage.
   *
   * @param voltage the voltage to set the motor to.
   */
  public void setVoltage(double voltage) {
    double clampedVoltage =
        GMath.clamp(
            voltage, -Constants.Arm.Rotation.MAX_VOLTAGE, Constants.Arm.Rotation.MAX_VOLTAGE);
    m_motor.setVoltage(clampedVoltage);
  }

  /**
   * Gets the current extension of the arm in meters.
   *
   * @return the current extension of the arm in meters.
   */
  public double getLength() {
    return Conversions.falconToMeters(
        m_motor.getSelectedSensorPosition(), Extension.LENGTH_PER_ROTATION, Extension.GEAR_RATIO);
  }

  /**
   * Locks the arm.
   *
   * <p>Disables movement by engaging the friction brake.
   */
  public void lock() {
    m_brake.set(false);
  }

  /**
   * Unlocks the arm.
   *
   * <p>Enables movement by disengaging the friction brake.
   */
  public void unlock() {
    m_brake.set(true);
  }

  /**
   * Locks the arm.
   *
   * <p>Disables movement by engaging the friction brake.
   */
  public void stop() {
    m_motor.stopMotor();
    lock();
  }

  private void configExtensionMotor() {
    m_motor.configFactoryDefault();
    m_motor.configAllSettings(Robot.ctreConfigs.armExtensionFXConfig);
    m_motor.setInverted(Extension.SHOULD_INVERT_MOTOR);
    m_motor.setNeutralMode(Extension.MOTOR_NEUTRAL_MODE);
    m_motor.setSelectedSensorPosition(0);
  }

  /**
   * Zeroes the extension motor encoder.
   *
   * <p>Resets the extension motor's internal encoder to zero. This ensures that future encoder
   * measurements correspond to the length of the arm.
   */
  private void setLength(double meters) {
    m_motor.setSelectedSensorPosition(
        Conversions.metersToFalcon(meters, Extension.LENGTH_PER_ROTATION, Extension.GEAR_RATIO));
  }

  @Override
  public void addToShuffleboard(ShuffleboardContainer container) {
    container.addDouble("Length (m)", this::getLength);
    container
        .addDouble("Speed (%)", m_motor::get)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -1.0, "max", 1.0));
    container.addBoolean("Unlocked?", m_brake::get);
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub

  }
}
