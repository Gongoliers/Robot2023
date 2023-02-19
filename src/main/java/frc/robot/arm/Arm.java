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

  private ArmState validate(ArmState state) {
    double minExtension = 0.0; // TODO
    double maxExtension = 0.0; // TODO
    double extension = MathUtil.clamp(state.extension, minExtension, maxExtension);
    Rotation2d angle = state.angle;
    return new ArmState(extension, angle);
  }

  public void set(ArmState state) {
    state = validate(state);
    setAngle(state.angle);
    setExtension(state.extension);
    m_desiredState = state;
  }

  private void setExtension(double extension) {
    double position =
        Conversions.MetersToFalcon(
            extension,
            Constants.Arm.EXTENSION_LENGTH_PER_ROTATION,
            Constants.Arm.EXTENSION_MOTOR_GEAR_RATIO);
    m_extensionMotor.set(ControlMode.Position, position);
  }

  private void setAngle(Rotation2d angle) {
    double position =
        Conversions.degreesToFalcon(angle.getDegrees(), Constants.Arm.ROTATION_MOTOR_GEAR_RATIO);
    m_rotationMotor.set(ControlMode.Position, position);
  }

  private double extension() {
    return Conversions.falconToMeters(
        m_extensionMotor.getSelectedSensorPosition(),
        Constants.Arm.EXTENSION_LENGTH_PER_ROTATION,
        Constants.Arm.EXTENSION_MOTOR_GEAR_RATIO);
  }

  private Rotation2d angle() {
    return Rotation2d.fromDegrees(
        Conversions.falconToDegrees(
            m_rotationMotor.getSelectedSensorPosition(), Constants.Arm.ROTATION_MOTOR_GEAR_RATIO));
  }

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
    SmartDashboard.putNumber("Actual Extension (m)", m_actualState.extension);
    SmartDashboard.putNumber("Actual Angle (deg)", m_actualState.angle.getDegrees());
  }
}
