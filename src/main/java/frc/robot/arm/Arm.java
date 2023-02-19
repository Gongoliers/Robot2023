package frc.robot.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Robot;

public class Arm {

  private TalonFX m_rotationMotor;
  private TalonFX m_extensionMotor;
  private CANCoder m_rotationCANCoder;

  public Arm() {
    m_rotationMotor = new TalonFX(Constants.Arm.ROTATION_MOTOR_CAN_ID, Constants.Arm.CANBUS_NAME);
    configExtensionMotor();

    m_extensionMotor = new TalonFX(Constants.Arm.EXTENSION_MOTOR_CAN_ID, Constants.Arm.CANBUS_NAME);
    configRotationMotor();

    m_rotationCANCoder =
        new CANCoder(Constants.Arm.ROTATION_CANCODER_CAN_ID, Constants.Arm.CANBUS_NAME);
    configRotationCANCoder();
  }

  public void set(ArmState state) {
    setAngle(state.angle);
    setExtension(state.extension);
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
}
