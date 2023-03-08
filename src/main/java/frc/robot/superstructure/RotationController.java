package frc.robot.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.thegongoliers.output.interfaces.Extendable;
import com.thegongoliers.output.interfaces.Lockable;
import com.thegongoliers.output.interfaces.Retractable;
import com.thegongoliers.output.interfaces.Stoppable;
import edu.wpi.first.math.geometry.Rotation2d;
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

public class RotationController extends SubsystemBase
    implements Stoppable, Lockable, Extendable, Retractable, TelemetrySubsystem {

  private WPI_TalonFX m_motor;
  private WPI_CANCoder m_cancoder;

  private Solenoid m_brake;

  private ArmState m_stowedState;
  private ArmState m_extendedState;

  public RotationController() {
    m_motor = new WPI_TalonFX(Constants.Arm.ROTATION_MOTOR_CAN_ID, Constants.Arm.CANBUS_NAME);
    configRotationMotor();

    m_cancoder =
        new WPI_CANCoder(Constants.Arm.ROTATION_CANCODER_CAN_ID, Constants.Arm.CANBUS_NAME);
    configRotationCANCoder();

    m_brake =
        new Solenoid(
            Constants.PNEUMATICS_HUB_ID,
            PneumaticsModuleType.REVPH,
            Constants.Arm.ROTATION_BRAKE_CHANNEL);

    lock();

    m_stowedState = Constants.Arm.States.STOWED;
    m_extendedState = Constants.Arm.States.STOWED;

    realignRotationSensor();

    addToShuffleboard(Shuffleboard.getTab("Arm"));
  }

  public void setSpeed(double percent) {
    m_motor.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Approach the desired state.
   *
   * @param extendedState the state to approach.
   */
  public void setExtendedState(ArmState extendedState) {
    m_extendedState = extendedState;
  }

  /**
   * Approaches the desired angle. Commands the angle motor's PID controller to approach the desired
   * angle.
   *
   * @param angle the angle to approach.
   */
  private void setGoal(Rotation2d angle) {
    double setpoint =
        Conversions.degreesToFalcon(angle.getDegrees(), Constants.Arm.ROTATION_MOTOR_GEAR_RATIO);
    m_motor.set(ControlMode.Position, setpoint);
  }

  /**
   * Get the current angle.
   *
   * @return the current angle.
   */
  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(
        Conversions.falconToDegrees(
            m_motor.getSelectedSensorPosition(), Constants.Arm.ROTATION_MOTOR_GEAR_RATIO));
  }

  private void configRotationMotor() {
    m_motor.configFactoryDefault();
    m_motor.configAllSettings(Robot.ctreConfigs.armRotationFXConfig);
    m_motor.setInverted(Constants.Arm.SHOULD_INVERT_ROTATION_MOTOR);
    m_motor.setNeutralMode(Constants.Arm.ROTATION_MOTOR_NEUTRAL_MODE);
    // TODO
  }

  private void configRotationCANCoder() {
    m_cancoder.configFactoryDefault();
    m_cancoder.configAllSettings(Robot.ctreConfigs.rotationCanCoderConfig);
  }

  /**
   * Realigns the rotation motor encoder to the CANCoder angle.
   *
   * <p>Resets the rotation motor's internal encoder to the CANCoder angle value. This ensures that
   * future encoder measurements will align with the angle of the arm.
   */
  private void realignRotationSensor() {
    m_motor.setSelectedSensorPosition(
        Conversions.degreesToFalcon(
            getCANCoderAngle().getDegrees(), Constants.Arm.ROTATION_MOTOR_GEAR_RATIO));
  }

  /**
   * Gets the angle of the CANCoder.
   *
   * @return the angle measured by the CANCoder.
   */
  private Rotation2d getCANCoderAngle() {
    double measurement = m_cancoder.getAbsolutePosition();
    double angle = measurement - Constants.Arm.CANCODER_OFFSET;
    return Rotation2d.fromDegrees(angle);
  }

  /**
   * Retracts the arm to the stowed position.
   *
   * <p>Note that this does not block functions of the subsystem; the PID controllers of each motor
   * runs.
   */
  @Override
  public void retract() {
    setGoal(m_stowedState.getAngle());
  }

  /**
   * Gets whether the arm is fully retracted ("stowed position").
   *
   * @return whether the arm is fully retracted ("stowed position").
   */
  @Override
  public boolean isRetracted() {
    return getAngle().equals(m_stowedState.getAngle());
  }

  /**
   * Extends the arm to the selected position.
   *
   * <p>Note that this does not block functions of the subsystem; the PID controllers of each motor
   * runs.
   */
  @Override
  public void extend() {
    setGoal(m_extendedState.getAngle());
  }

  /**
   * Gets whether the arm is fully extended.
   *
   * @return whether the arm is fully extended.
   */
  @Override
  public boolean isExtended() {
    return getAngle().equals(m_extendedState.getAngle());
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
        .addNumber("Extended Angle", () -> m_extendedState.getAngle().getDegrees())
        .withPosition(0, 0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", Constants.Arm.MIN_ANGLE, "max", Constants.Arm.MAX_ANGLE));

    extendedLayout.addBoolean("Is Extended?", this::isExtended).withPosition(0, 2);

    var stowedLayout = container.getLayout("Stowed State", BuiltInLayouts.kList);
    stowedLayout.withProperties(Map.of("Label position", "TOP")).withSize(2, 4).withPosition(2, 0);

    stowedLayout
        .addNumber("Stowed Angle", () -> m_stowedState.getAngle().getDegrees())
        .withPosition(0, 0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", Constants.Arm.MIN_ANGLE, "max", Constants.Arm.MAX_ANGLE));

    stowedLayout.addBoolean("Is Stowed?", this::isRetracted).withPosition(0, 3);

    var actualLayout = container.getLayout("Actual State", BuiltInLayouts.kList);
    actualLayout.withProperties(Map.of("Label position", "TOP")).withSize(2, 4).withPosition(4, 0);

    actualLayout
        .addNumber("Actual Angle", () -> getAngle().getDegrees())
        .withPosition(0, 0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", Constants.Arm.MIN_ANGLE, "max", Constants.Arm.MAX_ANGLE));

    var brakeLayout = container.getLayout("Brakes", BuiltInLayouts.kList);
    brakeLayout.withProperties(Map.of("Label position", "TOP")).withSize(2, 4).withPosition(6, 0);

    brakeLayout.addBoolean("Rotation Brake Active?", m_brake::get).withPosition(0, 0);
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub

  }
}
