package frc.robot.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.Arm.Rotation;
import frc.robot.Robot;

public class RotationController extends ProfiledPIDSubsystem {

  private final WPI_TalonFX m_motor;
  private final Solenoid m_brake;
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(Rotation.KS, Rotation.KG, Rotation.KV, Rotation.KA);

  public RotationController() {

    // initialPosition is the initial goal
    super(
        new ProfiledPIDController(Rotation.KP, 0, 0, Rotation.CONSTRAINTS),
        Constants.Arm.States.STOWED.getAngle().getDegrees());

    m_motor = new WPI_TalonFX(Rotation.MOTOR_ID, Constants.Arm.CANBUS_NAME);
    configRotationMotor();

    m_brake =
        new Solenoid(
            Constants.PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, Rotation.BRAKE_CHANNEL);

    lock();

    setMeasurement(Constants.Arm.States.STOWED.getAngle().getDegrees());

    Shuffleboard.getTab("a").addDouble("Degrees", () -> getMeasurement());
    Shuffleboard.getTab("a").addDouble("Mystery", () -> m_motor.getSelectedSensorPosition());
    Shuffleboard.getTab("a").addBoolean("Enabled?", () -> super.m_enabled);
  }

  /**
   * Manually drive the motor at the desired speed.
   *
   * @param percent the speed to drive the motor at.
   */
  public void drive(double percent) {
    m_motor.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Uses the output from the PIDController to drive the motor.
   *
   * @param output volts calculated by the PIDController.
   * @param setpoint the setpoint for the PIDController. Used for calculating feedforward.
   */
  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    SmartDashboard.putNumber("out", output);
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    double voltage =
        MathUtil.clamp(output + feedforward, -Rotation.MAX_VOLTAGE, Rotation.MAX_VOLTAGE);
    m_motor.setVoltage(voltage);
  }

  /**
   * Gets the current rotation of the arm in degrees.
   *
   * @return the current rotation of the arm in degrees.
   */
  @Override
  public double getMeasurement() {
    return Conversions.falconToDegrees(m_motor.getSelectedSensorPosition(), Rotation.GEAR_RATIO);
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
    m_motor.setInverted(Rotation.SHOULD_INVERT_MOTOR);
    m_motor.setNeutralMode(Rotation.MOTOR_NEUTRAL_MODE);
    // TODO
  }

  /**
   * Realigns the rotation motor encoder to the CANCoder angle.
   *
   * <p>Resets the rotation motor's internal encoder to the CANCoder angle value. This ensures that
   * future encoder measurements will align with the angle of the arm.
   */
  private void setMeasurement(double degrees) {
    m_motor.setSelectedSensorPosition(Conversions.degreesToFalcon(degrees, Rotation.GEAR_RATIO));
  }
}
