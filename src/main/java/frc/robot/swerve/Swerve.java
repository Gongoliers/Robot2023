package frc.robot.swerve;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.SwerveModule;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {

  private final SwerveDriveOdometry swerveOdometry;
  private final SwerveModule[] modules;
  private final Pigeon2 gyro;

  private final String[] moduleNameFromNumber =
      new String[] {"Front Left", "Front Right", "Back Left", "Back Right"};
  private double currentTurboScalar;

  public Swerve() {
    gyro = new Pigeon2(Constants.Swerve.PIGEON_ID, Constants.Swerve.CANBUS_NAME);
    gyro.configFactoryDefault();
    setYawZero();

    modules =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.FRONT_LEFT_MODULE.CONSTANTS),
          new SwerveModule(1, Constants.Swerve.FRONT_RIGHT_MODULE.CONSTANTS),
          new SwerveModule(2, Constants.Swerve.BACK_LEFT_MODULE.CONSTANTS),
          new SwerveModule(3, Constants.Swerve.BACK_RIGHT_MODULE.CONSTANTS)
        };

    /*
     * By pausing init for a second before setting module offsets, we avoid a bug
     * with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(1.0);
    resetModulesToAbsolute();

    SmartDashboard.putData("Reset Modules", resetModules());

    swerveOdometry =
        new SwerveDriveOdometry(Constants.Swerve.SWERVE_KINEMATICS, yaw(), positions());
    currentTurboScalar = Constants.Driver.NORMAL_SCALAR;
  }

  public void drive(
      Translation2d translation, Rotation2d rotation, boolean fieldRelative, boolean isOpenLoop) {

    Translation2d scaledTranslation = translation.times(currentTurboScalar);
    Rotation2d scaledRotation = rotation.times(currentTurboScalar);

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            scaledTranslation.getX(), scaledTranslation.getY(), scaledRotation.getRadians());
    // TODO Test whether field-relative driving works this way
    if (fieldRelative) speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, yaw());

    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.Swerve.LINEAR_SPEED_MAX);

    for (SwerveModule mod : modules) {
      mod.setDesiredState(swerveModuleStates[mod.number], isOpenLoop);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.LINEAR_SPEED_MAX);

    for (SwerveModule module : modules) {
      module.setDesiredState(desiredStates[module.number], false);
    }
  }

  public Pose2d pose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d toPose) {
    swerveOdometry.resetPosition(yaw(), positions(), toPose);
  }

  public SwerveModuleState[] states() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (SwerveModule module : modules) {
      states[module.number] = module.getState();
    }

    return states;
  }

  public SwerveModulePosition[] positions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule module : modules) {
      positions[module.number] = module.getPosition();
    }

    return positions;
  }

  public void setYawZero() {
    gyro.setYaw(0);
  }

  public Rotation2d yaw() {
    double yaw = gyro.getYaw();

    if (Constants.Swerve.SHOULD_INVERT_GYRO) {
      yaw = 360 - yaw;
    }

    return Rotation2d.fromDegrees(yaw);
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule module : modules) {
      module.resetToAbsolute();
    }
  }

  public CommandBase zeroGyro() {
    return runOnce(() -> setYawZero());
  }

  public CommandBase enableTurbo() {
    return this.runOnce(() -> currentTurboScalar = Constants.Driver.TURBO_SCALAR);
  }

  public CommandBase disableTurbo() {
    return this.runOnce(() -> currentTurboScalar = Constants.Driver.NORMAL_SCALAR);
  }

  public CommandBase resetModules() {
    return this.runOnce(() -> resetModulesToAbsolute());
  }

  @Override
  public void periodic() {
    swerveOdometry.update(yaw(), positions());

    for (SwerveModule module : modules) {
      String moduleName = moduleNameFromNumber[module.number];
      SmartDashboard.putNumber(moduleName + " Cancoder Angle", module.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          moduleName + " Integrated Encoder Angle", module.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(moduleName + " Velocity", module.getState().speedMetersPerSecond);
    }

    SmartDashboard.putNumber("Gyro Yaw", yaw().getDegrees());
    SmartDashboard.putNumber("Pose X", pose().getX());
    SmartDashboard.putNumber("Pose Y", pose().getY());
  }
}
