package frc.robot.superstructure;

import java.util.Map;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.fasterxml.jackson.databind.ser.std.NumberSerializers.DoubleSerializer;
import com.thegongoliers.math.GMath;
import com.thegongoliers.output.interfaces.Lockable;
import com.thegongoliers.output.interfaces.Stoppable;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.TelemetrySubsystem;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.Arm.Extension;
import frc.robot.Robot;

public class ExtensionController extends SubsystemBase
    implements Lockable, Stoppable, TelemetrySubsystem {

  private final RotationController m_rotationController;
  private final WPI_TalonFX m_motor;
  private final Solenoid m_brake;

  public ExtensionController(RotationController rotation) {

    m_rotationController = rotation;

    m_motor = new WPI_TalonFX(Extension.MOTOR_ID, Constants.Arm.CANBUS_NAME);
    configExtensionMotor();

    m_brake =
        new Solenoid(
            Constants.PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, Extension.BRAKE_CHANNEL);

    lock();

    // Assumes that the arm begins in the stowed state
    setLength(ArmState.STOWED.length);

    addToShuffleboard(Shuffleboard.getTab("Superstructure").getLayout("Extension", BuiltInLayouts.kList));
  }

  /**
   * Manually drive the motor at the desired speed.
   *
   * @param percent the speed to drive the motor at.
   */
  public void setMotor(double percent) {
    percent = GMath.clamp(percent, -1.0, 1.0);
    m_motor.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Gets the current extension of the arm in meters.
   *
   * @return the current extension of the arm in meters.
   */
  public double getLength() {
    return Conversions.falconToMeters(
        m_motor.getSelectedSensorPosition(), Extension.LENGTH_PER_ROTATION, Extension.GEAR_RATIO);
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
   * Stops the arm motor.
   */
  public void stop() {
    m_motor.stopMotor();
  }

  public Command drive(double percent, BooleanSupplier isFinished) {
    return this.runOnce(this::unlock).andThen(Commands.run(() -> setMotor(percent))).until(isFinished).andThen(Commands.runOnce(() -> { stop(); lock(); }));
  } 

  public Command extend() {
    return drive(Constants.Arm.Extension.MANUAL_EXTEND_SPEED, this::isExtended);
  }

  public Command retract() {
    return drive(Constants.Arm.Extension.MANUAL_RETRACT_SPEED, this::isRetracted);
  }

  private void configExtensionMotor() {
    m_motor.configFactoryDefault();
    m_motor.configAllSettings(Robot.ctreConfigs.armExtensionFXConfig);
    m_motor.setInverted(Extension.SHOULD_INVERT_MOTOR);
    m_motor.setNeutralMode(Extension.MOTOR_NEUTRAL_MODE);
  }

  /**
   * Zeroes the extension motor encoder.
   *
   * <p>Resets the extension motor's internal encoder to zero. This ensures that future encoder
   * measurements correspond to the length of the arm.
   */
  private void setLength(double meters) {
    m_motor.setSelectedSensorPosition(
        Conversions.metersToFalcon(meters, Extension.LENGTH_PER_ROTATION, Extension.GEAR_RATIO));
  }

  @Override
  public void periodic() {}

  @Override
  public void addToShuffleboard(ShuffleboardContainer container) {
    container.addDouble("Length (m)", this::getLength);
    container.addNumber("Max Length (m)", this::getLengthConstraint);
    container.addBoolean("Retracted to Min Length?", this::isRetracted);
    container.addBoolean("Extended to Max Length?", this::isExtended);
    container
        .addDouble("Speed (%)", m_motor::get)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -1.0, "max", 1.0));
    container.addBoolean("Unlocked?", m_brake::get);
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub
  }

  /**
   * Gets the maximum length of the arm for the current angle.
   * @return the maximum length of the arm.
   */
  private double getLengthConstraint() {
    double angle = m_rotationController.getAngle();
    return ArmMath.calculateLengthConstraint(angle);
  }

  public boolean isRetracted() {
    return getLength() <= Constants.Arm.Extension.MIN_EXTENSION_LENGTH;
  }

  public boolean isExtended() {
    return getLength() >= getLengthConstraint();
  }
}
