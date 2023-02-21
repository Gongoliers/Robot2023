package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
  private final CANCoder m_angleEncoder;

  // Offset between the actual zero heading and the CANCoder zero heading
  private final Rotation2d m_angleOffset;

  // Previous angle of the swerve module
  private Rotation2d m_previousAngle;

  // 
  private SwerveModuleState m_swerveModuleState = new SwerveModuleState();
  private SwerveModulePosition m_swerveModulePosition = new SwerveModulePosition();
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

    m_absoluteAngle = cancoderAngle();

    m_angleEncoder = new CANCoder(config.cancoderID, Constants.Swerve.CANBUS_NAME);
    configCANCoder();

    m_angleMotor = new WPI_TalonFX(config.angleMotorID, Constants.Swerve.CANBUS_NAME);
    configAngleMotor();

    m_driveMotor = new WPI_TalonFX(config.driveMotorID, Constants.Swerve.CANBUS_NAME);
    configDriveMotor();

    m_swerveModuleState = state();
    m_swerveModulePosition = position();
    m_previousAngle = encoderAngle();
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
    m_swerveModuleState = state();
    m_swerveModulePosition = position();
    m_absoluteAngle = cancoderAngle();
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
  public void setDesiredState(
      SwerveModuleState desiredState, boolean isOpenLoop) {
    // Use custom optimize function since CTRE is not a continuous controller
    desiredState = CTREModuleState.optimize(desiredState, state().angle);
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
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
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.LINEAR_SPEED_MAX;
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
    if (Math.abs(desiredState.speedMetersPerSecond) < (Constants.Swerve.LINEAR_SPEED_MAX * 0.01)) {
      angle = m_previousAngle;
    }

    m_angleMotor.set(
        ControlMode.Position,
        Conversions.degreesToFalcon(
            angle.getDegrees(), Constants.Swerve.ANGLE_MOTOR_GEAR_RATIO));

    m_previousAngle = angle;
  }

  /** Realign the angle encoders to the current angle of the wheel. */
  public void realignEncoderToCANCoder() {
    // Get the absolute angle from the CANCoder
    double cancoderAngle = cancoderAngle().getDegrees() - m_angleOffset.getDegrees();
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
  public SwerveModuleState state() {
    return new SwerveModuleState(
        Conversions.falconToMPS(
            m_driveMotor.getSelectedSensorVelocity(),
            Constants.Swerve.WHEEL_CIRCUMFERENCE,
            Constants.Swerve.DRIVE_MOTOR_GEAR_RATIO),
        encoderAngle());
  }

  /**
   * Get the current displacement of the module.
   *
   * @return the current displacement of the module.
   */
  public SwerveModulePosition position() {
    double position = 
        Conversions.falconToMeters(
            m_driveMotor.getSelectedSensorPosition(),
            Constants.Swerve.WHEEL_CIRCUMFERENCE,
            Constants.Swerve.DRIVE_MOTOR_GEAR_RATIO);
    return new SwerveModulePosition(position,
        state().angle);
  }

  /**
   * Get the current angle of the CANCoder. This is equivalent the angle of the wheel.
   *
   * @return the current angle of the CANCoder.
   */
  public Rotation2d cancoderAngle() {
    return Rotation2d.fromDegrees(m_angleEncoder.getAbsolutePosition());
  }

  /**
   * Get the current angle of the integrated encoder.
   *
   * @return the current angle of the integrated encoder.
   */
  private Rotation2d encoderAngle() {
    return Rotation2d.fromDegrees(
        Conversions.falconToDegrees(
            m_angleMotor.getSelectedSensorPosition(), Constants.Swerve.ANGLE_MOTOR_GEAR_RATIO));
  }
}
