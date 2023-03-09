package frc.robot.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.lib.math.Conversions;
import frc.robot.Robot;
import frc.robot.Constants.Arm.Extension;
import frc.robot.Constants;

public class ExtensionController extends ProfiledPIDSubsystem {

  private final WPI_TalonFX m_motor;
  private final Solenoid m_brake;
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          Extension.KS,
          Extension.KG,
          Extension.KV,
          Extension.KA);

  public ExtensionController() {

    super(
        new ProfiledPIDController(
            Extension.KP, 0, 0, Extension.CONSTRAINTS),
        0);

    m_motor = new WPI_TalonFX(Extension.MOTOR_ID, Constants.Arm.CANBUS_NAME);
    configExtensionMotor();

    m_brake =
        new Solenoid(
            Constants.PNEUMATICS_HUB_ID,
            PneumaticsModuleType.REVPH,
            Extension.BRAKE_CHANNEL);

    lock();

    // Assumes that the arm begins in the stowed state
    setPosition(0);
  }

  /**
   * Manually drive the motor at the desired speed.
   *
   * @param percent the speed to drive the motor at.
   */
  public void drive(double percent) {
    this.disable();
    m_motor.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    m_motor.setVoltage(output + feedforward);
  }

  /**
   * Get the current extension in meters.
   *
   * @return the current extension in meters.
   */
  @Override
  public double getMeasurement() {
    return Conversions.falconToMeters(
        m_motor.getSelectedSensorPosition(),
        Extension.LENGTH_PER_ROTATION,
        Extension.GEAR_RATIO);
  }

  /**
   * Locks the arm.
   *
   * <p>Disables movement by engaging the friction brake.
   */
  public void lock() {
    this.disable();
    m_brake.set(false);
  }

  /**
   * Unlocks the arm.
   *
   * <p>Enables movement by disengaging the friction brake.
   */
  public void unlock() {
    this.disable();
    m_brake.set(true);
  }

  /**
   * Locks the arm.
   *
   * <p>Disables movement by engaging the friction brake.
   */
  public void stop() {
    this.disable();
    m_motor.stopMotor();
    lock();
  }

  private void configExtensionMotor() {
    m_motor.configFactoryDefault();
    m_motor.configAllSettings(Robot.ctreConfigs.armExtensionFXConfig);
    m_motor.setInverted(Extension.SHOULD_INVERT_MOTOR);
    m_motor.setNeutralMode(Extension.MOTOR_NEUTRAL_MODE);
    m_motor.setSelectedSensorPosition(0);
    // TODO
  }

  /**
   * Zeroes the extension motor encoder.
   *
   * <p>Resets the extension motor's internal encoder to zero. This ensures that future encoder
   * measurements correspond to the length of the arm.
   */
  private void setPosition(double meters) {
    m_motor.setSelectedSensorPosition(
        Conversions.metersToFalcon(
            meters,
            Extension.LENGTH_PER_ROTATION,
            Extension.GEAR_RATIO));
  }
}
