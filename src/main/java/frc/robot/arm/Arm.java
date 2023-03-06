package frc.robot.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.thegongoliers.output.interfaces.Extendable;
import com.thegongoliers.output.interfaces.Lockable;
import com.thegongoliers.output.interfaces.Retractable;
import com.thegongoliers.output.interfaces.Stoppable;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.TelemetrySubsystem;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.Map;

public class Arm extends SubsystemBase
    implements Stoppable, Lockable, Extendable, Retractable, TelemetrySubsystem {

  private TalonFX m_rotationMotor;
  private TalonFX m_extensionMotor;
  private CANCoder m_rotationCANCoder;

  private Solenoid m_rotationBrake;
  private Solenoid m_extensionBrake;

  private ArmState m_stowedState;
  private ArmState m_extendedState;
  private ArmState m_actualState;

  public Arm() {
    m_rotationMotor = new TalonFX(Constants.Arm.ROTATION_MOTOR_CAN_ID, Constants.Arm.CANBUS_NAME);
    configExtensionMotor();

    m_extensionMotor = new TalonFX(Constants.Arm.EXTENSION_MOTOR_CAN_ID, Constants.Arm.CANBUS_NAME);
    configRotationMotor();

    m_rotationCANCoder =
        new CANCoder(Constants.Arm.ROTATION_CANCODER_CAN_ID, Constants.Arm.CANBUS_NAME);
    configRotationCANCoder();

    m_rotationBrake =
        new Solenoid(PneumaticsModuleType.REVPH, Constants.Arm.ROTATION_BRAKE_CHANNEL);
    m_extensionBrake =
        new Solenoid(PneumaticsModuleType.REVPH, Constants.Arm.EXTENSION_BRAKE_CHANNEL);
    lock();

    m_stowedState = Constants.Arm.States.STOWED;
    // TODO Select default extended state
    m_extendedState = Constants.Arm.States.STOWED;

    // Automatically start in the stowed state
    m_actualState = m_stowedState;
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
        MathUtil.clamp(
            desiredState.getLength(), getMinimumLength(angle), getMaximumLength(angle));
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
   * @param desiredState the state to approach.
   */
  public void set(ArmState desiredState) {
    m_extendedState = validate(desiredState);
  }

  /**
   * Approach the desired extension length.
   *
   * @param extension the extension length (in meters) to approach.
   */
  private void setExtension(double extension) {
    double setpoint =
        Conversions.metersToFalcon(
            extension,
            Constants.Arm.EXTENSION_LENGTH_PER_ROTATION,
            Constants.Arm.EXTENSION_MOTOR_GEAR_RATIO);
    m_extensionMotor.set(ControlMode.Position, setpoint);
  }

  /**
   * Approach the desired angle.
   *
   * @param angle the angle to approach.
   */
  private void setAngle(Rotation2d angle) {
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
    return Rotation2d.fromDegrees(
        Conversions.falconToDegrees(
            m_rotationMotor.getSelectedSensorPosition(), Constants.Arm.ROTATION_MOTOR_GEAR_RATIO));
  }

  /**
   * Get the current arm state.
   *
   * @return the current arm state.
   */
  public ArmState state() {
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
    m_rotationCANCoder.configAllSettings(Robot.ctreConfigs.armCanCoderConfig);
  }

  @Override
  public void periodic() {
    // Update the current state
    m_actualState = state();
  }

  @Override
  public void retract() {
    setAngle(m_stowedState.getAngle());
    setExtension(m_stowedState.getLength());
  }

  @Override
  public boolean isRetracted() {
    return m_actualState.equals(Constants.Arm.States.STOWED);
  }

  @Override
  public void extend() {
    setAngle(m_extendedState.getAngle());
    setExtension(m_extendedState.getLength());
  }

  @Override
  public boolean isExtended() {
    return (isRetracted() == false) && m_actualState.equals(m_extendedState);
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
      .withProperties(Map.of("Label position", "TOP"));

    extendedLayout
        .addNumber("Extended Angle", () -> m_extendedState.getAngle().getDegrees())
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", Constants.Arm.MIN_ANGLE, "max", Constants.Arm.MAX_ANGLE));

    extendedLayout
        .addNumber("Extended Length", () -> m_extendedState.getLength())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(
            Map.of(
                "min",
                getMinimumLength(m_extendedState.getAngle()),
                "max",
                getMaximumLength(m_extendedState.getAngle())));

    extendedLayout
      .addBoolean("Is Extended?", this::isExtended);

    var stowedLayout = container.getLayout("Stowed State", BuiltInLayouts.kList);
    stowedLayout
      .withProperties(Map.of("Label position", "TOP"));

    stowedLayout
        .addNumber("Stowed Angle", () -> m_stowedState.getAngle().getDegrees())
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", Constants.Arm.MIN_ANGLE, "max", Constants.Arm.MAX_ANGLE));

    stowedLayout
        .addNumber("Stowed Length", () -> m_stowedState.getLength())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(
            Map.of(
                "min",
                getMinimumLength(m_stowedState.getAngle()),
                "max",
                getMaximumLength(m_stowedState.getAngle())));

    stowedLayout
      .addBoolean("Is Stowed?", this::isRetracted);

    var actualLayout = container.getLayout("Actual State", BuiltInLayouts.kList);
    actualLayout
      .withProperties(Map.of("Label position", "TOP"));

    actualLayout
        .addNumber("Actual Angle", () -> m_actualState.getAngle().getDegrees())
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", Constants.Arm.MIN_ANGLE, "max", Constants.Arm.MAX_ANGLE));

    actualLayout
        .addNumber("Actual Length", () -> m_actualState.getLength())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(
            Map.of(
                "min",
                getMinimumLength(m_actualState.getAngle()),
                "max",
                getMaximumLength(m_actualState.getAngle())));

    var brakeLayout = container.getLayout("Brakes", BuiltInLayouts.kList);
    brakeLayout
      .addBoolean("Rotation Brake Active?", () -> m_rotationBrake.get());

    brakeLayout
      .addBoolean("Extension Brake Active?", () -> m_extensionBrake.get());
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub

  }
}
