package frc.robot.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.thegongoliers.output.interfaces.Extendable;
import com.thegongoliers.output.interfaces.Lockable;
import com.thegongoliers.output.interfaces.Retractable;
import com.thegongoliers.output.interfaces.Stoppable;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.TelemetrySubsystem;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Robot;

import java.util.Map;

public class Arm extends SubsystemBase
    implements Stoppable, Lockable, Extendable, Retractable, TelemetrySubsystem {

  private WPI_TalonFX m_rotationMotor;
  private WPI_TalonFX m_extensionMotor;
  private WPI_CANCoder m_rotationCANCoder;
  private WPI_CANCoder m_extensionCANCoder;

  private Solenoid m_rotationBrake;
  private Solenoid m_extensionBrake;

  private ArmState m_stowedState;
  private ArmState m_extendedState;
  private ArmState m_state;

  private Timer m_simTimer;
  private double m_simPreviousTimestamp, m_simAngle, m_simLength;

  public Arm() {
    if (!Robot.isReal()) {
      m_simTimer = new Timer();
      m_simTimer.start();
      m_simPreviousTimestamp = 0;
    }

    m_rotationMotor =
        new WPI_TalonFX(Constants.Arm.ROTATION_MOTOR_CAN_ID, Constants.Arm.CANBUS_NAME);
    configRotationMotor();

    m_extensionMotor =
        new WPI_TalonFX(Constants.Arm.EXTENSION_MOTOR_CAN_ID, Constants.Arm.CANBUS_NAME);
    configExtensionMotor();

    m_rotationCANCoder =
        new WPI_CANCoder(Constants.Arm.ROTATION_CANCODER_CAN_ID, Constants.Arm.CANBUS_NAME);
    configRotationCANCoder();

    m_extensionCANCoder =
        new WPI_CANCoder(Constants.Arm.ROTATION_CANCODER_CAN_ID, Constants.Arm.CANBUS_NAME);
    configExtensionCANCoder();

    m_rotationBrake =
        new Solenoid(PneumaticsModuleType.REVPH, Constants.Arm.ROTATION_BRAKE_CHANNEL);
    m_extensionBrake =
        new Solenoid(PneumaticsModuleType.REVPH, Constants.Arm.EXTENSION_BRAKE_CHANNEL);
    lock();

    realignRotationSensor();

    m_stowedState = Constants.Arm.States.STOWED;
    // TODO Select default extended state
    m_extendedState = Constants.Arm.States.STOWED;

    // Assumes that the arm begins the match in the stowed state
    zeroExtensionLength();
    m_state = m_stowedState;

    addToShuffleboard(Shuffleboard.getTab("Arm"));
  }

  /**
   * Validate a state. This ensures that the arm will not crash into any part of the superstructure
   * or field when approaching the desired state.
   *
   * @param desiredState the state to be validated.
   * @return a new state that is guarenteed to be a valid state.
   */
  private ArmState validate(ArmState desiredState) {
    double degrees =
        MathUtil.clamp(
            desiredState.getAngle().getDegrees(), Constants.Arm.MIN_ANGLE, Constants.Arm.MAX_ANGLE);
    Rotation2d angle = Rotation2d.fromDegrees(degrees);

    double extension =
        MathUtil.clamp(desiredState.getLength(), getMinimumLength(angle), getMaximumLength(angle));
    return new ArmState(extension, angle);
  }

  private double getMinimumLength(Rotation2d angle) {
    return Constants.Arm.kAngleToMinLength.get(angle.getDegrees());
  }

  private double getMaximumLength(Rotation2d angle) {
    return Constants.Arm.kAngleToMaxLength.get(angle.getDegrees());
  }

  /**
   * Approach the desired state.
   *
   * @param extendedState the state to approach.
   */
  public void setExtendedState(ArmState extendedState) {
    m_extendedState = validate(extendedState);
  }

  /**
   * Approaches the extension desired length. Commands the extension motor's PID controller to
   * approach the extension length.
   *
   * @param extension the extension length (in meters) to approach.
   */
  private void setExtension(double extension) {
    if (!Robot.isReal()) {
      m_simLength = extension;
    }

    double setpoint =
        Conversions.metersToFalcon(
            extension,
            Constants.Arm.EXTENSION_LENGTH_PER_ROTATION,
            Constants.Arm.EXTENSION_MOTOR_GEAR_RATIO);
    m_extensionMotor.set(ControlMode.Position, setpoint);
  }

  /**
   * Approaches the desired angle. Commands the angle motor's PID controller to approach the desired
   * angle.
   *
   * @param angle the angle to approach.
   */
  private void setAngle(Rotation2d angle) {
    if (!Robot.isReal()) {
      m_simAngle = angle.getDegrees();
    }

    double setpoint =
        Conversions.degreesToFalcon(angle.getDegrees(), Constants.Arm.ROTATION_MOTOR_GEAR_RATIO);
    m_rotationMotor.set(ControlMode.Position, setpoint);
  }

  /**
   * Get the current extension in meters.
   *
   * @return the current extension in meters.
   */
  private double getExtension() {
    if (!Robot.isReal()) {
      return m_simLength;
    }

    return Conversions.falconToMeters(
        m_extensionMotor.getSelectedSensorPosition(),
        Constants.Arm.EXTENSION_LENGTH_PER_ROTATION,
        Constants.Arm.EXTENSION_MOTOR_GEAR_RATIO);
  }

  /**
   * Get the current angle.
   *
   * @return the current angle.
   */
  private Rotation2d getAngle() {
    if (!Robot.isReal()) {
      return Rotation2d.fromDegrees(m_simAngle);
    }

    return Rotation2d.fromDegrees(
        Conversions.falconToDegrees(
            m_rotationMotor.getSelectedSensorPosition(), Constants.Arm.ROTATION_MOTOR_GEAR_RATIO));
  }

  /**
   * Get the current arm state.
   *
   * @return the current arm state.
   */
  public ArmState getState() {
    return new ArmState(getExtension(), getAngle());
  }

  private void configRotationMotor() {
    m_rotationMotor.configFactoryDefault();
    m_rotationMotor.configAllSettings(Robot.ctreConfigs.armRotationFXConfig);
    m_rotationMotor.setInverted(Constants.Arm.SHOULD_INVERT_ROTATION_MOTOR);
    m_rotationMotor.setNeutralMode(Constants.Arm.ROTATION_MOTOR_NEUTRAL_MODE);
    // TODO
  }

  private void configExtensionMotor() {
    m_extensionMotor.configFactoryDefault();
    m_extensionMotor.configAllSettings(Robot.ctreConfigs.armExtensionFXConfig);
    m_extensionMotor.setInverted(Constants.Arm.SHOULD_INVERT_EXTENSION_MOTOR);
    m_extensionMotor.setNeutralMode(Constants.Arm.EXTENSION_MOTOR_NEUTRAL_MODE);
    m_extensionMotor.setSelectedSensorPosition(0);
    // TODO
  }

  private void configRotationCANCoder() {
    m_rotationCANCoder.configFactoryDefault();
    m_rotationCANCoder.configAllSettings(Robot.ctreConfigs.rotationCanCoderConfig);
  }

  private void configExtensionCANCoder() {
    m_rotationCANCoder.configFactoryDefault();
    m_rotationCANCoder.configAllSettings(Robot.ctreConfigs.extensionCanCoderConfig);
  }

  private void realignRotationSensor() {
    m_rotationMotor.setSelectedSensorPosition(
        Conversions.degreesToFalcon(
            getCANCoderAngle().getDegrees(), Constants.Arm.ROTATION_MOTOR_GEAR_RATIO));
  }

  private void zeroExtensionLength() {
    double stowedLength = Constants.Arm.States.STOWED.getLength();
    m_extensionMotor.setSelectedSensorPosition(
        Conversions.metersToFalcon(
            stowedLength,
            Constants.Arm.EXTENSION_LENGTH_PER_ROTATION,
            Constants.Arm.EXTENSION_MOTOR_GEAR_RATIO));
  }

  private Rotation2d getCANCoderAngle() {
    double measurement = m_rotationCANCoder.getAbsolutePosition();
    double angle = measurement - Constants.Arm.CANCODER_OFFSET;
    return Rotation2d.fromDegrees(angle);
  }

  @Override
  public void periodic() {
    if (!Robot.isReal()) {
      m_simPreviousTimestamp = m_simTimer.get();
    }
    // Update the current state
    m_state = getState();
  }

  @Override
  public void retract() {
    setAngle(m_stowedState.getAngle());
    setExtension(m_stowedState.getLength());
  }

  @Override
  public boolean isRetracted() {
    return m_state.equals(Constants.Arm.States.STOWED);
  }

  @Override
  public void extend() {
    setAngle(m_extendedState.getAngle());
    setExtension(m_extendedState.getLength());
  }

  @Override
  public boolean isExtended() {
    return (isRetracted() == false) && m_state.equals(m_extendedState);
  }

  @Override
  public void lock() {
    m_extensionBrake.set(true);
    m_rotationBrake.set(true);
  }

  @Override
  public void unlock() {
    m_extensionBrake.set(false);
    m_rotationBrake.set(false);
  }

  @Override
  public void stop() {
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

    extendedLayout
        .addNumber("Extended Length", m_extendedState::getLength)
        .withPosition(0, 1)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(
            Map.of(
                "min",
                getMinimumLength(m_extendedState.getAngle()),
                "max",
                getMaximumLength(m_extendedState.getAngle())));

    extendedLayout.addBoolean("Is Extended?", this::isExtended).withPosition(0, 2);

    var stowedLayout = container.getLayout("Stowed State", BuiltInLayouts.kList);
    stowedLayout.withProperties(Map.of("Label position", "TOP")).withSize(2, 4).withPosition(2, 0);

    stowedLayout
        .addNumber("Stowed Angle", () -> m_stowedState.getAngle().getDegrees())
        .withPosition(0, 0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", Constants.Arm.MIN_ANGLE, "max", Constants.Arm.MAX_ANGLE));

    stowedLayout
        .addNumber("Stowed Length", m_stowedState::getLength)
        .withPosition(0, 1)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(
            Map.of(
                "min",
                getMinimumLength(m_stowedState.getAngle()),
                "max",
                getMaximumLength(m_stowedState.getAngle())));

    stowedLayout.addBoolean("Is Stowed?", this::isRetracted).withPosition(0, 3);

    var actualLayout = container.getLayout("Actual State", BuiltInLayouts.kList);
    actualLayout.withProperties(Map.of("Label position", "TOP")).withSize(2, 4).withPosition(4, 0);

    actualLayout
        .addNumber("Actual Angle", () -> m_state.getAngle().getDegrees())
        .withPosition(0, 0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", Constants.Arm.MIN_ANGLE, "max", Constants.Arm.MAX_ANGLE));

    actualLayout
        .addNumber("Actual Length", m_state::getLength)
        .withPosition(0, 1)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(
            Map.of(
                "min",
                getMinimumLength(m_state.getAngle()),
                "max",
                getMaximumLength(m_state.getAngle())));

    var brakeLayout = container.getLayout("Brakes", BuiltInLayouts.kList);
    brakeLayout.withProperties(Map.of("Label position", "TOP")).withSize(2, 4).withPosition(6, 0);

    brakeLayout.addBoolean("Rotation Brake Active?", m_rotationBrake::get).withPosition(0, 0);

    brakeLayout.addBoolean("Extension Brake Active?", m_extensionBrake::get).withPosition(0, 1);
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub

  }
}
