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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Swerve extends SubsystemBase {

  private final SwerveDriveOdometry m_swerveOdometry;
  private final SwerveModule[] m_modules;
  private final Pigeon2 m_gyro;

  private SwerveModuleState[] m_swerveModuleStates;
  private ChassisSpeeds m_chassisSpeeds;

  private final Field2d m_field;

  private Timer m_simTimer;
  private double m_simPreviousTimestamp, m_simYaw;

  public Swerve() {
    if (!Robot.isReal()) {
      m_simTimer = new Timer();
      m_simTimer.start();
      m_simPreviousTimestamp = 0;
    }

    m_gyro = new Pigeon2(Constants.Swerve.PIGEON_ID, Constants.Swerve.CANBUS_NAME);
    m_gyro.configFactoryDefault();

    setYawZero();

    m_modules =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.FRONT_LEFT_MODULE.CONFIG),
          new SwerveModule(1, Constants.Swerve.FRONT_RIGHT_MODULE.CONFIG),
          new SwerveModule(2, Constants.Swerve.BACK_LEFT_MODULE.CONFIG),
          new SwerveModule(3, Constants.Swerve.BACK_RIGHT_MODULE.CONFIG)
        };

    /*
     * By pausing init for a second before setting module offsets, we avoid a bug
     * with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(1.0);
    realignEncodersToCANCoder();

    m_swerveOdometry =
        new SwerveDriveOdometry(Constants.Swerve.SWERVE_KINEMATICS, yaw(), positions());

    m_swerveModuleStates = states();
    m_chassisSpeeds = Constants.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(m_swerveModuleStates);

    SwerveTelemetry.createShuffleboardTab(this, "Swerve");
    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);
  }

  /** Stop all modules. */
  public void stop() {
    for (var module : m_modules) {
      module.stop();
    }
  }

  /**
   * Direct the swerve modules to drive the robot.
   *
   * @param translation a vector containing the desired X and Y velocity of the chassis.
   * @param rotation the desired rotational velocity of the chassis.
   * @param isOpenLoop whether the swerve modules are driven in open loop (velocity direct from
   *     driver) or closed loop (velocity controlled by PID).
   */
  public void drive(Translation2d translation, Rotation2d rotation, boolean isOpenLoop) {

    // Create a ChassisSpeeds object to contain the desired velocities
    ChassisSpeeds speeds =
        new ChassisSpeeds(translation.getX(), translation.getY(), rotation.getRadians());

    // Transform the desired velocities to be relative to the robot heading
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, yaw());

    // Convert the desired velocities to module states
    SwerveModuleState[] desiredStates =
        Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

    // Renormalize the wheel speeds to avoid exceeding the maximum chassis speed
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.LINEAR_SPEED_MAX);

    // Set the desired state for each module
    for (var module : m_modules) {
      module.setDesiredState(desiredStates[module.id], isOpenLoop);
    }
  }

  /**
   * Set the swerve module states. Used for setting the module states for autonomous.
   *
   * @param desiredStates the module states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Renormalize the wheel speeds to avoid exceeding the maximum chassis speed
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.LINEAR_SPEED_MAX);

    // Set the desired state for each module
    for (var module : m_modules) {
      module.setDesiredState(desiredStates[module.id], false);
    }
  }

  /**
   * Get the current pose of the robot. The displacements are measured in meters.
   *
   * @return the current pose (position) of the robot.
   */
  public Pose2d pose() {
    return m_swerveOdometry.getPoseMeters();
  }

  /**
   * Reset the robot's current pose. This overrides the previous pose. After this calling function,
   * the robot will believe it is at the pose parameter.
   *
   * @param toPose the pose the robot will be reset to.
   */
  public void resetOdometry(Pose2d toPose) {
    m_swerveOdometry.resetPosition(yaw(), positions(), toPose);
  }

  public SwerveModule module(int moduleNumber) {
    return m_modules[moduleNumber];
  }

  /**
   * Get the current swerve module states.
   *
   * @return the current swerve module states.
   */
  public SwerveModuleState[] states() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    // Get the state of each module
    for (var module : m_modules) {
      states[module.id] = module.state();
    }

    return states;
  }

  /**
   * Get the current swerve module positions.
   *
   * @return the current swerve module positions.
   */
  public SwerveModulePosition[] positions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    // Get the position of each module
    for (var module : m_modules) {
      positions[module.id] = module.position();
    }

    return positions;
  }

  /**
   * Get the current chassis speeds.
   *
   * @return the current chassis speeds.
   */
  public ChassisSpeeds speeds() {
    return m_chassisSpeeds;
  }

  /**
   * Set the gyro's current yaw to be zero. This makes the robot's previous yaw the new zero point
   * of the robot. Driving will now be relative to the yaw the robot was prior to this call.
   */
  private void setYawZero() {
    if (!Robot.isReal()) {
      m_simYaw = 0;
    }
    m_gyro.setYaw(0);
  }

  /**
   * Get the current yaw (counter-clockwise / clockwise rotation) of the robot.
   *
   * @return the current yaw of the robot.
   */
  private Rotation2d yaw() {
    double yaw;
    if (!Robot.isReal()) {
      yaw = m_simYaw;
    } else {
      yaw = m_gyro.getYaw();

      if (Constants.Swerve.SHOULD_INVERT_GYRO) {
        yaw = 360 - yaw;
      }
    }
    return new Rotation2d(yaw);
  }

  /**
   * Instruct all of the modules to realign their angle encoders to the angle value provided by the
   * CANCoder.
   */
  public void realignEncodersToCANCoder() {
    for (var module : m_modules) {
      module.realignEncoderToCANCoder();
    }
  }

  @Override
  public void periodic() {
    if (!Robot.isReal()) {
      ChassisSpeeds speeds = Constants.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(states());
      double deltaTime = m_simTimer.get() - m_simPreviousTimestamp;
      m_simYaw += speeds.omegaRadiansPerSecond * deltaTime;
      m_simPreviousTimestamp = m_simTimer.get();
    }

    // Update the swerve odometry to the latest position measurements
    m_swerveOdometry.update(yaw(), positions());

    // Update the Field display on SmartDashboard
    m_field.setRobotPose(pose());
    SmartDashboard.putData("Field", m_field);

    // Update the current module states and chassis speeds
    m_swerveModuleStates = states();
    m_chassisSpeeds = Constants.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(m_swerveModuleStates);

    SmartDashboard.putNumberArray(
        "ModuleStates",
        new double[] {
          m_swerveModuleStates[0].angle.getDegrees(), m_swerveModuleStates[0].speedMetersPerSecond,
          m_swerveModuleStates[1].angle.getDegrees(), m_swerveModuleStates[1].speedMetersPerSecond,
          m_swerveModuleStates[2].angle.getDegrees(), m_swerveModuleStates[2].speedMetersPerSecond,
          m_swerveModuleStates[3].angle.getDegrees(), m_swerveModuleStates[3].speedMetersPerSecond,
        });

    double velocity =
        new Translation2d(m_chassisSpeeds.vxMetersPerSecond, m_chassisSpeeds.vyMetersPerSecond)
            .getNorm();
    SmartDashboard.putNumber("Velocity", velocity);
  }
}
