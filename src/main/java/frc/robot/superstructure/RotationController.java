package frc.robot.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.thegongoliers.output.interfaces.Lockable;
import com.thegongoliers.output.interfaces.Stoppable;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ArmState;
import frc.lib.TelemetrySubsystem;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.Arm.Rotation;
import frc.robot.Robot;
import java.util.Map;

public class RotationController extends SubsystemBase
    implements Lockable, Stoppable, TelemetrySubsystem {

  private final WPI_TalonFX m_motor;
  private final Solenoid m_brake;

  public RotationController() {

    m_motor = new WPI_TalonFX(Rotation.MOTOR_ID, Constants.Arm.CANBUS_NAME);
    configRotationMotor();

    m_brake =
        new Solenoid(
            Constants.PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, Rotation.BRAKE_CHANNEL);

    lock();

    setAngle(ArmState.STOWED.getAngle());

    addToShuffleboard(Shuffleboard.getTab("Superstructure").getLayout("Rotation", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0));
  }

  /**
   * Manually drive the motor at the desired speed.
   *
   * @param percent the speed to drive the motor at.
   */
  public void drive(double percent) {
    m_motor.set(ControlMode.PercentOutput, percent);
  }

  public double getAngle() {
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

  /** Stops the motor. */
  public void stop() {
    m_motor.stopMotor();
  }

  private void configRotationMotor() {
    m_motor.configFactoryDefault();
    m_motor.configAllSettings(Robot.ctreConfigs.armRotationFXConfig);
    m_motor.setInverted(Rotation.SHOULD_INVERT_MOTOR);
    m_motor.setNeutralMode(Rotation.MOTOR_NEUTRAL_MODE);
  }

  /**
   * Realigns the rotation motor encoder to the CANCoder angle.
   *
   * <p>Resets the rotation motor's internal encoder to the CANCoder angle value. This ensures that
   * future encoder measurements will align with the angle of the arm.
   */
  private void setAngle(double degrees) {
    m_motor.setSelectedSensorPosition(Conversions.degreesToFalcon(degrees, Rotation.GEAR_RATIO));
  }

  @Override
  public void addToShuffleboard(ShuffleboardContainer container) {
    container.addDouble("Angle (deg)", this::getAngle).withPosition(0, 0);
    container.addBoolean("Lowered to Min Angle?", this::isLowered);
    container.addBoolean("Raised to Max Angle?", this::isRaised);
    container
        .addDouble("Speed (%)", m_motor::get)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -1.0, "max", 1.0)).withPosition(0, 1);
    container.addBoolean("Unlocked?", m_brake::get).withPosition(0, 2);
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub
  }

  public boolean isLowered() {
    return getAngle() <= Constants.Arm.Rotation.MIN_ANGLE;
  }

  public boolean isRaised() {
    return getAngle() >= Constants.Arm.Rotation.MAX_ANGLE;
  }
}
