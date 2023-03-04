package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ctre.CTREModuleState;
import frc.lib.math.Conversions;
import frc.lib.swerve.SwerveModuleConfig;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule extends SubsystemBase {
  /** The identification number for this module. */
  public final int id;

  private final WPI_TalonFX m_angleMotor;
  private final WPI_TalonFX m_driveMotor;
  private final WPI_CANCoder m_angleEncoder;

  // Offset between the actual zero heading and the CANCoder zero heading
  private final Rotation2d m_angleOffset;

  // Previous angle of the swerve module
  private Rotation2d m_previousAngle;

  private Timer m_simTimer;
  private double m_simPreviousTimestamp, m_simAngle, m_simDistance, m_simDeltaTime, m_simSpeed;

  // Store the current state
  @SuppressWarnings("unused")
  private SwerveModuleState m_state = new SwerveModuleState();

  @SuppressWarnings("unused")
  private SwerveModulePosition m_position = new SwerveModulePosition();

  @SuppressWarnings("unused")
  private Rotation2d m_absoluteAngle = new Rotation2d();

  // The feed-forward calculator for the drive motor
  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.DRIVE_MOTOR_KS,
          Constants.Swerve.DRIVE_MOTOR_KV,
          Constants.Swerve.DRIVE_MOTOR_KA);

  public SwerveModule(int moduleNumber, SwerveModuleConfig config) {
    id = moduleNumber;
    m_angleOffset = config.angleOffset;

    m_angleEncoder = new WPI_CANCoder(config.cancoderID, Constants.Swerve.CANBUS_NAME);
    configCANCoder();

    m_absoluteAngle = getCANCoderAngle();

    m_angleMotor = new WPI_TalonFX(config.angleMotorID, Constants.Swerve.CANBUS_NAME);
    configAngleMotor();

    m_driveMotor = new WPI_TalonFX(config.driveMotorID, Constants.Swerve.CANBUS_NAME);
    configDriveMotor();

    m_state = getState();
    m_position = getPosition();
    m_previousAngle = getEncoderAngle();

    if (!Robot.isReal()) {
      m_simTimer = new Timer();
      m_simTimer.start();
      m_simPreviousTimestamp = m_simTimer.get();
      m_simAngle = 0;
      m_simSpeed = 0;
      m_simDistance = 0;
    }
  }

  /** Configure the CANCoder using the default configuration. */
  private void configCANCoder() {
    m_angleEncoder.configFactoryDefault();
    m_angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  /** Configure the angle motor using the supplied constants. */
  private void configAngleMotor() {
    m_angleMotor.configFactoryDefault();
    m_angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
    m_angleMotor.setInverted(Constants.Swerve.SHOULD_INVERT_ANGLE_MOTOR);
    m_angleMotor.setNeutralMode(Constants.Swerve.ANGLE_MOTOR_NEUTRAL_MODE);
    realignEncoderToCANCoder();
  }

  /** Configure the drive motor using the supplied constants. */
  private void configDriveMotor() {
    m_driveMotor.configFactoryDefault();
    m_driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
    m_driveMotor.setInverted(Constants.Swerve.SHOULD_INVERT_DRIVE_MOTOR);
    m_driveMotor.setNeutralMode(Constants.Swerve.DRIVE_MOTOR_NEUTRAL_MODE);
    m_driveMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    m_state = getState();
    m_position = getPosition();
    m_absoluteAngle = getCANCoderAngle();
  }

  public void stop() {
    m_driveMotor.stopMotor();
    m_angleMotor.stopMotor();
  }

  /**
   * Set the desired state of this module.
   *
   * @param desiredState the desired speed and angle for this module.
   * @param isOpenLoop whether the swerve modules are driven in open loop (velocity direct from
   *     driver) or closed loop (velocity controlled by PID).
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Use custom optimize function since CTRE is not a continuous controller
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);

    if (!Robot.isReal()) {
      m_simAngle = desiredState.angle.getDegrees();
      m_simSpeed = desiredState.speedMetersPerSecond;

      m_simDeltaTime = m_simTimer.get() - m_simPreviousTimestamp;
      m_simDistance += (m_simSpeed * m_simDeltaTime);
      m_simPreviousTimestamp = m_simTimer.get();
    }
  }

  /**
   * Set the desired speed for this module.
   *
   * @param desiredState the desired state for this module.
   * @param isOpenLoop whether the swerve modules are driven in open loop (velocity direct from
   *     driver) or closed loop (velocity controlled by PID).
   */
  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
      m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity =
          Conversions.MPSToFalcon(
              desiredState.speedMetersPerSecond,
              Constants.Swerve.WHEEL_CIRCUMFERENCE,
              Constants.Swerve.DRIVE_MOTOR_GEAR_RATIO);
      m_driveMotor.set(
          ControlMode.Velocity,
          velocity,
          DemandType.ArbitraryFeedForward,
          m_feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  /**
   * Set the desired angle for this module.
   *
   * @param desiredState the desired state for this module.
   */
  private void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle = desiredState.angle;

    // Don't rotate module if speed is less than 1%
    if (Math.abs(desiredState.speedMetersPerSecond) < (Constants.Swerve.MAX_SPEED * 0.01)) {
      angle = m_previousAngle;
    }

    m_angleMotor.set(
        ControlMode.Position,
        Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.ANGLE_MOTOR_GEAR_RATIO));

    m_previousAngle = angle;
  }

  /** Realign the angle encoders to the current angle of the wheel. */
  public void realignEncoderToCANCoder() {
    // Get the absolute angle from the CANCoder
    double cancoderAngle = getCANCoderAngle().getDegrees() - m_angleOffset.getDegrees();
    // Update the heading of the angle encoder
    double encoderAngle =
        Conversions.degreesToFalcon(cancoderAngle, Constants.Swerve.ANGLE_MOTOR_GEAR_RATIO);
    m_angleMotor.setSelectedSensorPosition(encoderAngle);
  }

  /**
   * Get the current state of the module.
   *
   * @return the current state of the module.
   */
  public SwerveModuleState getState() {
    double speedMetersPerSecond;
    Rotation2d angle;
    if (!Robot.isReal()) {
      speedMetersPerSecond = m_simSpeed;
      angle = Rotation2d.fromDegrees(m_simAngle);
    } else {
      speedMetersPerSecond =
          Conversions.falconToMPS(
              m_driveMotor.getSelectedSensorVelocity(),
              Constants.Swerve.WHEEL_CIRCUMFERENCE,
              Constants.Swerve.DRIVE_MOTOR_GEAR_RATIO);
      angle = getEncoderAngle();
    }
    return new SwerveModuleState(speedMetersPerSecond, angle);
  }

  /**
   * Get the current displacement of the module.
   *
   * @return the current displacement of the module.
   */
  public SwerveModulePosition getPosition() {
    double distanceMeters;
    Rotation2d angle;
    if (!Robot.isReal()) {
      distanceMeters = m_simDistance;
      angle = Rotation2d.fromDegrees(m_simAngle);
    } else {
      distanceMeters =
          Conversions.falconToMeters(
              m_driveMotor.getSelectedSensorPosition(),
              Constants.Swerve.WHEEL_CIRCUMFERENCE,
              Constants.Swerve.DRIVE_MOTOR_GEAR_RATIO);
      angle = getState().angle;
    }
    return new SwerveModulePosition(distanceMeters, angle);
  }

  /**
   * Get the current angle of the CANCoder. This is equivalent the angle of the wheel.
   *
   * @return the current angle of the CANCoder.
   */
  public Rotation2d getCANCoderAngle() {
    return Rotation2d.fromDegrees(m_angleEncoder.getAbsolutePosition());
  }

  /**
   * Get the current angle of the integrated encoder.
   *
   * @return the current angle of the integrated encoder.
   */
  public Rotation2d getEncoderAngle() {
    return Rotation2d.fromDegrees(
        Conversions.falconToDegrees(
            m_angleMotor.getSelectedSensorPosition(), Constants.Swerve.ANGLE_MOTOR_GEAR_RATIO));
  }
}
