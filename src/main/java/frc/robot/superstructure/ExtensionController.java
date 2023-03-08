package frc.robot.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.thegongoliers.math.GMath;
import com.thegongoliers.output.interfaces.Extendable;
import com.thegongoliers.output.interfaces.Lockable;
import com.thegongoliers.output.interfaces.Retractable;
import com.thegongoliers.output.interfaces.Stoppable;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ArmState;
import frc.lib.TelemetrySubsystem;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.Map;

public class ExtensionController extends SubsystemBase
    implements Stoppable, Lockable, Extendable, Retractable, TelemetrySubsystem {

  private final WPI_TalonFX m_motor;

  private final Solenoid m_brake;

  private ArmState m_stowedState;
  private ArmState m_extendedState;

  public ExtensionController() {

    m_motor = new WPI_TalonFX(Constants.Arm.EXTENSION_MOTOR_CAN_ID, Constants.Arm.CANBUS_NAME);
    configExtensionMotor();

    m_brake =
        new Solenoid(
            Constants.PNEUMATICS_HUB_ID,
            PneumaticsModuleType.REVPH,
            Constants.Arm.EXTENSION_BRAKE_CHANNEL);

    lock();

    m_stowedState = Constants.Arm.States.STOWED;
    m_extendedState = Constants.Arm.States.STOWED;

    // Assumes that the arm begins the match in the stowed state
    zeroExtensionLength();

    addToShuffleboard(Shuffleboard.getTab("Arm"));
  }

  public void setSpeed(double percent) {
    m_motor.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Sets what the future (extended) state should be.
   *
   * @param extendedState the state to approach.
   */
  public void setExtendedState(ArmState extendedState) {
    m_extendedState = extendedState;
  }

  /**
   * Get the current extension in meters.
   *
   * @return the current extension in meters.
   */
  private double getLength() {
    return Conversions.falconToMeters(
        m_motor.getSelectedSensorPosition(),
        Constants.Arm.EXTENSION_LENGTH_PER_ROTATION,
        Constants.Arm.EXTENSION_MOTOR_GEAR_RATIO);
  }

  private void configExtensionMotor() {
    m_motor.configFactoryDefault();
    m_motor.configAllSettings(Robot.ctreConfigs.armExtensionFXConfig);
    m_motor.setInverted(Constants.Arm.SHOULD_INVERT_EXTENSION_MOTOR);
    m_motor.setNeutralMode(Constants.Arm.EXTENSION_MOTOR_NEUTRAL_MODE);
    m_motor.setSelectedSensorPosition(0);
    // TODO
  }

  /**
   * Zeroes the extension motor encoder.
   *
   * <p>Resets the extension motor's internal encoder to zero. This ensures that future encoder
   * measurements correspond to the length of the arm.
   */
  private void zeroExtensionLength() {
    double stowedLength = Constants.Arm.States.STOWED.getLength();
    m_motor.setSelectedSensorPosition(
        Conversions.metersToFalcon(
            stowedLength,
            Constants.Arm.EXTENSION_LENGTH_PER_ROTATION,
            Constants.Arm.EXTENSION_MOTOR_GEAR_RATIO));
  }

  /**
   * Approaches the extension desired length. Commands the extension motor's PID controller to
   * approach the extension length.
   *
   * @param length the extension length (in meters) to approach.
   */
  private void setGoal(double length) {
    double setpoint =
        Conversions.metersToFalcon(
            length,
            Constants.Arm.EXTENSION_LENGTH_PER_ROTATION,
            Constants.Arm.EXTENSION_MOTOR_GEAR_RATIO);
    m_motor.set(ControlMode.Position, setpoint);
  }

  /**
   * Retracts the arm to the stowed position.
   *
   * <p>Note that this does not block functions of the subsystem; the PID controllers of each motor
   * runs.
   */
  @Override
  public void retract() {
    double retractedLength = m_stowedState.getLength();
    setGoal(retractedLength);
  }

  /**
   * Gets whether the arm is fully retracted ("stowed position").
   *
   * @return whether the arm is fully retracted ("stowed position").
   */
  @Override
  public boolean isRetracted() {
    return GMath.approximately(getLength(), m_stowedState.getLength());
  }

  /**
   * Extends the arm to the selected position.
   *
   * <p>Note that this does not block functions of the subsystem; the PID controllers of each motor
   * runs.
   */
  @Override
  public void extend() {
    double extendedLength = m_extendedState.getLength();
    setGoal(extendedLength);
  }

  /**
   * Gets whether the arm is fully extended.
   *
   * @return whether the arm is fully extended.
   */
  @Override
  public boolean isExtended() {
    return GMath.approximately(getLength(), m_extendedState.getLength());
  }

  /**
   * Locks the arm.
   *
   * <p>Disables movement by engaging the friction brake.
   */
  @Override
  public void lock() {
    m_brake.set(true);
  }

  /**
   * Unlocks the arm.
   *
   * <p>Enables movement by disengaging the friction brake.
   */
  @Override
  public void unlock() {
    m_brake.set(false);
  }

  /**
   * Locks the arm.
   *
   * <p>Disables movement by engaging the friction brake.
   */
  @Override
  public void stop() {
    m_motor.set(0.0);
    lock();
  }

  @Override
  public void addToShuffleboard(ShuffleboardContainer container) {
    var extendedLayout = container.getLayout("Extended State", BuiltInLayouts.kList);
    extendedLayout
        .withProperties(Map.of("Label position", "TOP"))
        .withSize(2, 4)
        .withPosition(0, 0);

    extendedLayout
        .addNumber("Extended Length", m_extendedState::getLength)
        .withPosition(0, 1)
        .withWidget(BuiltInWidgets.kNumberBar);

    extendedLayout.addBoolean("Is Extended?", this::isExtended).withPosition(0, 2);

    var stowedLayout = container.getLayout("Stowed State", BuiltInLayouts.kList);
    stowedLayout.withProperties(Map.of("Label position", "TOP")).withSize(2, 4).withPosition(2, 0);

    stowedLayout
        .addNumber("Stowed Length", m_stowedState::getLength)
        .withPosition(0, 1)
        .withWidget(BuiltInWidgets.kNumberBar);

    stowedLayout.addBoolean("Is Stowed?", this::isRetracted).withPosition(0, 3);

    var actualLayout = container.getLayout("Actual State", BuiltInLayouts.kList);
    actualLayout.withProperties(Map.of("Label position", "TOP")).withSize(2, 4).withPosition(4, 0);

    actualLayout
        .addNumber("Actual Length", this::getLength)
        .withPosition(0, 1)
        .withWidget(BuiltInWidgets.kNumberBar);

    var brakeLayout = container.getLayout("Brakes", BuiltInLayouts.kList);
    brakeLayout.withProperties(Map.of("Label position", "TOP")).withSize(2, 4).withPosition(6, 0);

    brakeLayout.addBoolean("Extension Brake Active?", m_brake::get).withPosition(0, 1);
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub

  }
}
