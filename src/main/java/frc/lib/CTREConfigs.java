package frc.lib;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import frc.robot.Constants;

public final class CTREConfigs {

  public TalonFXConfiguration armRotationFXConfig;
  public TalonFXConfiguration armExtensionFXConfig;
  public CANCoderConfiguration rotationCanCoderConfig;

  public CTREConfigs() {
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
