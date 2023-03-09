package frc.robot.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Robot;

public class RotationController extends ProfiledPIDSubsystem {

  private final WPI_TalonFX m_motor;
  private final WPI_CANCoder m_cancoder;
  private final Solenoid m_brake;
  private final ArmFeedforward m_feedforward = new ArmFeedforward(
    Constants.Arm.ROTATION_KS,
    Constants.Arm.ROTATION_KG,
    Constants.Arm.ROTATION_KV,
    Constants.Arm.ROTATION_KA
  );

  public RotationController() {

    // FIXME Match initialPosition to CANCoder
    super(
      new ProfiledPIDController(Constants.Arm.ROTATION_MOTOR_KP, 0, 0, Constants.Arm.ROTATION_CONTRAINTS),
      0
    );

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

    setPosition(getCANCoderAngle());
  }

  /**
   * Manually drive the motor at the desired speed.
   *
   * @param percent the speed to drive the motor at.
   */
  public void drive(double percent) {
    m_motor.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    m_motor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    return Conversions.falconToDegrees(
            m_motor.getSelectedSensorPosition(), Constants.Arm.ROTATION_MOTOR_GEAR_RATIO);
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
  private void setPosition(Rotation2d rotation) {
    m_motor.setSelectedSensorPosition(
        Conversions.degreesToFalcon(
            rotation.getDegrees(), Constants.Arm.ROTATION_MOTOR_GEAR_RATIO));
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

}
