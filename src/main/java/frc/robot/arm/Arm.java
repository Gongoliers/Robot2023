package frc.robot.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Robot;

public class Arm extends SubsystemBase {

  private TalonFX m_rotationMotor;
  private TalonFX m_extensionMotor;
  private CANCoder m_rotationCANCoder;

  private ArmState m_desiredState;
  private ArmState m_actualState;

  public Arm() {
    m_rotationMotor = new TalonFX(Constants.Arm.ROTATION_MOTOR_CAN_ID, Constants.Arm.CANBUS_NAME);
    configExtensionMotor();

    m_extensionMotor = new TalonFX(Constants.Arm.EXTENSION_MOTOR_CAN_ID, Constants.Arm.CANBUS_NAME);
    configRotationMotor();

    m_rotationCANCoder =
        new CANCoder(Constants.Arm.ROTATION_CANCODER_CAN_ID, Constants.Arm.CANBUS_NAME);
    configRotationCANCoder();
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
            desiredState.angle().getDegrees(), Constants.Arm.MIN_ANGLE, Constants.Arm.MAX_ANGLE);
    Rotation2d angle = Rotation2d.fromDegrees(degrees);

    double minExtension = Constants.Arm.kAngleToMinLength.get(angle.getDegrees());
    double maxExtension = Constants.Arm.kAngleToMaxLength.get(angle.getDegrees());

    double extension = MathUtil.clamp(desiredState.extensionLength(), minExtension, maxExtension);
    return new ArmState(extension, angle);
  }

  /**
   * Approach the desired state.
   *
   * @param desiredState the state to approach.
   */
  public void set(ArmState desiredState) {
    desiredState = validate(desiredState);

    setAngle(desiredState.angle());
    setExtension(desiredState.extensionLength());

    m_desiredState = desiredState;
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
  private double extension() {
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
  private Rotation2d angle() {
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
    return new ArmState(extension(), angle());
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

    // Publish info to SmartDashboard
    // TODO Port to ShuffleBoard
    SmartDashboard.putNumber("Desired Extension (m)", m_desiredState.extensionLength());
    SmartDashboard.putNumber("Desired Angle (deg)", m_desiredState.angle().getDegrees());
    SmartDashboard.putNumber("Actual Extension (m)", m_actualState.extensionLength());
    SmartDashboard.putNumber("Actual Angle (deg)", m_actualState.angle().getDegrees());
  }
}
