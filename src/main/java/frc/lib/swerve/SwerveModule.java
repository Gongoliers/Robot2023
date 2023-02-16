package frc.lib.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.ctre.CTREModuleState;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
  /** The identification number for this module. */
  public final int id;

  private final TalonFX m_angleMotor;
  private final TalonFX m_driveMotor;
  private final CANCoder m_angleEncoder;

  // Offset between the actual zero heading and the CANCoder zero heading
  private final Rotation2d m_angleOffset;
  // State to return to when stopped
  private final SwerveModuleState m_stoppedState;

  // The feed-forward calculator for the drive motor
  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.DRIVE_MOTOR_KS,
          Constants.Swerve.DRIVE_MOTOR_KV,
          Constants.Swerve.DRIVE_MOTOR_KA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    id = moduleNumber;
    m_angleOffset = moduleConstants.angleOffset;
    m_stoppedState = new SwerveModuleState(0, moduleConstants.angleStop);

    m_angleEncoder = new CANCoder(moduleConstants.cancoderID, Constants.Swerve.CANBUS_NAME);
    configCANCoder();

    m_angleMotor = new TalonFX(moduleConstants.angleMotorID, Constants.Swerve.CANBUS_NAME);
    configAngleMotor();

    m_driveMotor = new TalonFX(moduleConstants.driveMotorID, Constants.Swerve.CANBUS_NAME);
    configDriveMotor();
  }

  /**
   * Set the desired state of this module.
   *
   * @param desiredState the desired speed and angle for this module.
   * @param isOpenLoop whether the swerve modules are driven in open loop (velocity direct from
   *     driver) or closed loop (velocity controlled by PID).
   * @param ableToStop whether the swerve module is able to stop.
   */
  public void setDesiredState(
      SwerveModuleState desiredState, boolean isOpenLoop, boolean ableToStop) {
    // Use custom optimize function since CTRE is not a continuous controller
    desiredState = CTREModuleState.optimize(desiredState, state().angle);
    // TODO Add a check here to ensure that the speed is zero
    if (ableToStop) {
      desiredState = m_stoppedState;
    }
    setAngle(desiredState.angle);
    setSpeed(desiredState.speedMetersPerSecond, isOpenLoop);
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
    return new SwerveModulePosition(
        Conversions.falconToMeters(
            m_driveMotor.getSelectedSensorPosition(),
            Constants.Swerve.WHEEL_CIRCUMFERENCE,
            Constants.Swerve.DRIVE_MOTOR_GEAR_RATIO),
        encoderAngle());
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
   * Set the desired speed for this module.
   *
   * @param desiredSpeed the desired speed for this module.
   * @param isOpenLoop whether the swerve modules are driven in open loop (velocity direct from
   *     driver) or closed loop (velocity controlled by PID).
   */
  private void setSpeed(double desiredSpeed, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredSpeed / Constants.Swerve.LINEAR_SPEED_MAX;
      m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity =
          Conversions.MPSToFalcon(
              desiredSpeed,
              Constants.Swerve.WHEEL_CIRCUMFERENCE,
              Constants.Swerve.DRIVE_MOTOR_GEAR_RATIO);
      m_driveMotor.set(
          ControlMode.Velocity,
          velocity,
          DemandType.ArbitraryFeedForward,
          m_feedforward.calculate(desiredSpeed));
    }
  }

  /**
   * Set the desired angle for this module.
   *
   * @param desiredAngle the angle for this module.
   */
  private void setAngle(Rotation2d desiredAngle) {
    m_angleMotor.set(
        ControlMode.Position,
        Conversions.degreesToFalcon(
            desiredAngle.getDegrees(), Constants.Swerve.ANGLE_MOTOR_GEAR_RATIO));
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
}
