package frc.lib.ctre;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import frc.robot.Constants;

public final class CTREConfigs {
  public TalonFXConfiguration swerveAngleFXConfig;
  public TalonFXConfiguration swerveDriveFXConfig;
  public CANCoderConfiguration swerveCanCoderConfig;

  public TalonFXConfiguration armRotationFXConfig;
  public TalonFXConfiguration armExtensionFXConfig;
  public CANCoderConfiguration rotationCanCoderConfig;

  public CTREConfigs() {
    swerveAngleFXConfig = new TalonFXConfiguration();
    swerveDriveFXConfig = new TalonFXConfiguration();
    swerveCanCoderConfig = new CANCoderConfiguration();

    /* Swerve Angle Motor Configurations */
    SupplyCurrentLimitConfiguration angleSupplyLimit =
        new SupplyCurrentLimitConfiguration(
            Constants.Swerve.SHOULD_CURRENT_LIMIT_ANGLE_MOTOR,
            Constants.Swerve.ANGLE_MOTOR_CONTINUOUS_CURRENT_MAX,
            Constants.Swerve.ANGLE_MOTOR_PEAK_CURRENT_MAX,
            Constants.Swerve.ANGLE_MOTOR_PEAK_CURRENT_DURATION);

    swerveAngleFXConfig.slot0.kP = Constants.Swerve.ANGLE_MOTOR_KP;
    swerveAngleFXConfig.slot0.kI = Constants.Swerve.ANGLE_MOTOR_KI;
    swerveAngleFXConfig.slot0.kD = Constants.Swerve.ANGLE_MOTOR_KD;
    swerveAngleFXConfig.slot0.kF = Constants.Swerve.ANGLE_MOTOR_KF;
    swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

    /* Swerve Drive Motor Configuration */
    SupplyCurrentLimitConfiguration driveSupplyLimit =
        new SupplyCurrentLimitConfiguration(
            Constants.Swerve.SHOULD_CURRENT_LIMIT_DRIVE_MOTOR,
            Constants.Swerve.DRIVE_MOTOR_CONTINUOUS_CURRENT_MAX,
            Constants.Swerve.DRIVE_MOTOR_PEAK_CURRENT_MAX,
            Constants.Swerve.DRIVE_MOTOR_PEAK_CURRENT_DURATION);

    swerveDriveFXConfig.slot0.kP = Constants.Swerve.DRIVE_MOTOR_KP;
    swerveDriveFXConfig.slot0.kI = Constants.Swerve.DRIVE_MOTOR_KI;
    swerveDriveFXConfig.slot0.kD = Constants.Swerve.DRIVE_MOTOR_KD;
    swerveDriveFXConfig.slot0.kF = Constants.Swerve.DRIVE_MOTOR_KF;
    swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
    swerveDriveFXConfig.openloopRamp = Constants.Swerve.OPEN_LOOP_RAMP_DURATION;
    swerveDriveFXConfig.closedloopRamp = Constants.Swerve.CLOSED_LOOP_RAMP_DURATION;

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    swerveCanCoderConfig.sensorDirection = Constants.Swerve.SHOULD_INVERT_CANCODER;
    swerveCanCoderConfig.initializationStrategy =
        SensorInitializationStrategy.BootToAbsolutePosition;
    swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    armRotationFXConfig = new TalonFXConfiguration();
    armExtensionFXConfig = new TalonFXConfiguration();
    rotationCanCoderConfig = new CANCoderConfiguration();

    // Arm rotation motor config
    SupplyCurrentLimitConfiguration rotationSupplyLimit =
        new SupplyCurrentLimitConfiguration(
            Constants.Arm.Rotation.SHOULD_CURRENT_LIMIT,
            Constants.Arm.Rotation.CONTINUOUS_CURRENT_MAX,
            Constants.Arm.Rotation.PEAK_CURRENT_MAX,
            Constants.Arm.Rotation.PEAK_CURRENT_DURATION);

    armRotationFXConfig.slot0.kP = Constants.Arm.Rotation.KP;
    armRotationFXConfig.slot0.kI = Constants.Arm.Rotation.KI;
    armRotationFXConfig.slot0.kD = Constants.Arm.Rotation.KD;
    armRotationFXConfig.supplyCurrLimit = rotationSupplyLimit;

    // Arm extension motor config
    SupplyCurrentLimitConfiguration extensionSupplyLimit =
        new SupplyCurrentLimitConfiguration(
            Constants.Arm.Extension.SHOULD_CURRENT_LIMIT,
            Constants.Arm.Extension.CONTINUOUS_CURRENT_MAX,
            Constants.Arm.Extension.PEAK_CURRENT_MAX,
            Constants.Arm.Extension.PEAK_CURRENT_DURATION);

    armExtensionFXConfig.slot0.kP = Constants.Arm.Extension.KP;
    armExtensionFXConfig.slot0.kI = Constants.Arm.Extension.KI;
    armExtensionFXConfig.slot0.kD = Constants.Arm.Extension.KD;
    armExtensionFXConfig.supplyCurrLimit = extensionSupplyLimit;

    // Arm rotation CANCoder config
    rotationCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    rotationCanCoderConfig.sensorDirection = Constants.Arm.Rotation.SHOULD_INVERT_CANCODER;
    rotationCanCoderConfig.initializationStrategy =
        SensorInitializationStrategy.BootToAbsolutePosition;
    rotationCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
  }
}
