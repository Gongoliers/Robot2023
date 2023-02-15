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
  public int number;
  private Rotation2d angleOffset;
  private Rotation2d previousAngle;

  private TalonFX angleMotor;
  private TalonFX driveMotor;
  private CANCoder angleEncoder;

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.DRIVE_MOTOR_KS,
          Constants.Swerve.DRIVE_MOTOR_KV,
          Constants.Swerve.DRIVE_MOTOR_KA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    number = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    angleEncoder = new CANCoder(moduleConstants.cancoderID, Constants.Swerve.CANBUS_NAME);
    configCANCoder();

    angleMotor = new TalonFX(moduleConstants.angleMotorID, Constants.Swerve.CANBUS_NAME);
    configAngleMotor();

    driveMotor = new TalonFX(moduleConstants.driveMotorID, Constants.Swerve.CANBUS_NAME);
    configDriveMotor();

    previousAngle = state().angle;
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
    desiredState = CTREModuleState.optimize(desiredState, state().angle);
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  /** Realign the angle encoders to the current angle of the wheel. */
  public void realignEncoderToCANCoder() {
    // Get the absolute angle from the CANCoder
    double cancoderAngle = cancoderAngle().getDegrees() - angleOffset.getDegrees();
    // Update the heading of the angle encoder
    double encoderAngle =
        Conversions.degreesToFalcon(cancoderAngle, Constants.Swerve.ANGLE_MOTOR_GEAR_RATIO);
    angleMotor.setSelectedSensorPosition(encoderAngle);
  }

  /**
   * Get the current state of the module.
   * @return the current state of the module.
   */
  public SwerveModuleState state() {
    return new SwerveModuleState(
        Conversions.falconToMPS(
            driveMotor.getSelectedSensorVelocity(),
            Constants.Swerve.WHEEL_CIRCUMFERENCE,
            Constants.Swerve.DRIVE_MOTOR_GEAR_RATIO),
        encoderAngle());
  }

  /**
   * Get the current displacement of the module.
   * @return the current displacement of the module.
   */
  public SwerveModulePosition position() {
    return new SwerveModulePosition(
        Conversions.falconToMeters(
            driveMotor.getSelectedSensorPosition(),
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
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  /**
   * Set the desired speed of this module.
   *
   * @param desiredState the desired speed and angle for this module.
   * @param isOpenLoop whether the swerve modules are driven in open loop (velocity direct from
   *     driver) or closed loop (velocity controlled by PID).
   */
  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.LINEAR_SPEED_MAX;
      driveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity =
          Conversions.MPSToFalcon(
              desiredState.speedMetersPerSecond,
              Constants.Swerve.WHEEL_CIRCUMFERENCE,
              Constants.Swerve.DRIVE_MOTOR_GEAR_RATIO);
      driveMotor.set(
          ControlMode.Velocity,
          velocity,
          DemandType.ArbitraryFeedForward,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  /**
   * Set the desired angle of this module.
   *
   * @param desiredState the desired speed and angle for this module.
   */
  private void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle;
    // Prevent changing the angle if the speed is low enough to prevent jittering.
    if (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.LINEAR_SPEED_MAX * 0.01)) {
      // Since the speed is low enough, we should maintain the previous angle
      angle = previousAngle;
    } else {
      angle = desiredState.angle;
    }

    angleMotor.set(
        ControlMode.Position,
        Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.ANGLE_MOTOR_GEAR_RATIO));
    previousAngle = angle;
  }

  /**
   * Get the current angle of the integrated encoder.
   *
   * @return the current angle of the integrated encoder.
   */
  private Rotation2d encoderAngle() {
    return Rotation2d.fromDegrees(
        Conversions.falconToDegrees(
            angleMotor.getSelectedSensorPosition(), Constants.Swerve.ANGLE_MOTOR_GEAR_RATIO));
  }

  /**
   * Configure the CANCoder using the default configuration.
   */
  private void configCANCoder() {
    angleEncoder.configFactoryDefault();
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  /**
   * Configure the angle motor using the supplied constants.
   */
  private void configAngleMotor() {
    angleMotor.configFactoryDefault();
    angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
    angleMotor.setInverted(Constants.Swerve.SHOULD_INVERT_ANGLE_MOTOR);
    angleMotor.setNeutralMode(Constants.Swerve.ANGLE_MOTOR_NEUTRAL_MODE);
    realignEncoderToCANCoder();
  }

  /**
   * Configure the drive motor using the supplied constants.
   */
  private void configDriveMotor() {
    driveMotor.configFactoryDefault();
    driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
    driveMotor.setInverted(Constants.Swerve.SHOULD_INVERT_DRIVE_MOTOR);
    driveMotor.setNeutralMode(Constants.Swerve.DRIVE_MOTOR_NEUTRAL_MODE);
    driveMotor.setSelectedSensorPosition(0);
  }
}
