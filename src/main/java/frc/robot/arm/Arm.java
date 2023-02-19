package frc.robot.arm;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import frc.robot.Constants;

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

  void configRotationMotor() {
    // TODO
  }

  void configExtensionMotor() {
    // TODO
  }

  void configRotationCANCoder() {
    // TODO
  }
}
