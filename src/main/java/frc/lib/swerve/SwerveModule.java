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

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID, Constants.Swerve.CANBUS_NAME);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new TalonFX(moduleConstants.angleMotorID, Constants.Swerve.CANBUS_NAME);
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new TalonFX(moduleConstants.driveMotorID, Constants.Swerve.CANBUS_NAME);
    configDriveMotor();

    previousAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    /* is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.LINEAR_SPEED_MAX;
      driveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity =
          Conversions.mpsToFalcon(
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

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(
        Conversions.falconToDegrees(
            angleMotor.getSelectedSensorPosition(), Constants.Swerve.ANGLE_MOTOR_GEAR_RATIO));
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public void resetToAbsolute() {
    double absolutePosition =
        Conversions.degreesToFalcon(
            getCanCoder().getDegrees() - angleOffset.getDegrees(),
            Constants.Swerve.ANGLE_MOTOR_GEAR_RATIO);
    angleMotor.setSelectedSensorPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    angleMotor.configFactoryDefault();
    angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
    angleMotor.setInverted(Constants.Swerve.SHOULD_INVERT_ANGLE_MOTOR);
    angleMotor.setNeutralMode(Constants.Swerve.ANGLE_MOTOR_NEUTRAL_MODE);
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.configFactoryDefault();
    driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
    driveMotor.setInverted(Constants.Swerve.SHOULD_INVERT_DRIVE_MOTOR);
    driveMotor.setNeutralMode(Constants.Swerve.DRIVE_MOTOR_NEUTRAL_MODE);
    driveMotor.setSelectedSensorPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Conversions.falconToMPS(
            driveMotor.getSelectedSensorVelocity(),
            Constants.Swerve.WHEEL_CIRCUMFERENCE,
            Constants.Swerve.DRIVE_MOTOR_GEAR_RATIO),
        getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Conversions.falconToMeters(
            driveMotor.getSelectedSensorPosition(),
            Constants.Swerve.WHEEL_CIRCUMFERENCE,
            Constants.Swerve.DRIVE_MOTOR_GEAR_RATIO),
        getAngle());
  }
}
