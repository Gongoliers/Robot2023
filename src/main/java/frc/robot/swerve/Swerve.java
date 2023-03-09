package frc.robot.swerve;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.TelemetrySubsystem;
import frc.robot.Constants;
import frc.robot.Robot;

public class Swerve extends SubsystemBase implements TelemetrySubsystem {

  private final SwerveDriveOdometry m_swerveOdometry;
  private final SwerveModule[] m_modules;
  private final WPI_Pigeon2 m_gyro;

  private SwerveModuleState[] m_swerveModuleStates;
  private ChassisSpeeds m_chassisSpeeds;

  private Timer m_simTimer;
  private double m_simPreviousTimestamp, m_simYaw;

  public Swerve() {
    if (!Robot.isReal()) {
      m_simTimer = new Timer();
      m_simTimer.start();
      m_simPreviousTimestamp = 0;
    }

    m_gyro = new WPI_Pigeon2(Constants.Swerve.PIGEON_ID, Constants.Swerve.CANBUS_NAME);
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
        new SwerveDriveOdometry(Constants.Swerve.SWERVE_KINEMATICS, getYaw(), getPositions());

    m_swerveModuleStates = getStates();
    m_chassisSpeeds = Constants.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(m_swerveModuleStates);

    addToShuffleboard(Shuffleboard.getTab("Swerve"));
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
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw());

    // Convert the desired velocities to module states
    SwerveModuleState[] desiredStates =
        Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

    // Renormalize the wheel speeds to avoid exceeding the maximum chassis speed
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

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
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

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
  public Pose2d getPose() {
    return m_swerveOdometry.getPoseMeters();
  }

  /**
   * Reset the robot's current pose. This overrides the previous pose. After this calling function,
   * the robot will believe it is at the pose parameter.
   *
   * @param toPose the pose the robot will be reset to.
   */
  public void resetOdometry(Pose2d toPose) {
    m_swerveOdometry.resetPosition(getYaw(), getPositions(), toPose);
  }

  @Override
  public void addToShuffleboard(ShuffleboardContainer container) {
    container.addNumber("Heading", () -> this.getPose().getRotation().getDegrees());
    container.addNumber("Odometry X", () -> this.getPose().getX());
    container.addNumber("Odometry Y", () -> this.getPose().getY());

    for (var module : m_modules) {
      module.addToShuffleboard(container);
    }
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub
  }

  public SwerveModule getModule(int moduleNumber) {
    return m_modules[moduleNumber];
  }

  /**
   * Get the current swerve module states.
   *
   * @return the current swerve module states.
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    // Get the state of each module
    for (var module : m_modules) {
      states[module.id] = module.getState();
    }

    return states;
  }

  /**
   * Get the current swerve module positions.
   *
   * @return the current swerve module positions.
   */
  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    // Get the position of each module
    for (var module : m_modules) {
      positions[module.id] = module.getPosition();
    }

    return positions;
  }

  /**
   * Get the current chassis speeds.
   *
   * @return the current chassis speeds.
   */
  public ChassisSpeeds getSpeeds() {
    return m_chassisSpeeds;
  }

  /**
   * Set the gyro's current yaw to be zero. This makes the robot's previous yaw the new zero point
   * of the robot. Driving will now be relative to the yaw the robot was prior to this call.
   */
  public void setYawZero() {
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
  public Rotation2d getYaw() {
    if (!Robot.isReal()) {
      return new Rotation2d(m_simYaw);
    }
    Rotation2d yaw = m_gyro.getRotation2d();

    if (Constants.Swerve.SHOULD_INVERT_GYRO) {
      yaw = yaw.unaryMinus();
    }
    return yaw;
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
      ChassisSpeeds speeds = Constants.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(getStates());
      double deltaTime = m_simTimer.get() - m_simPreviousTimestamp;
      m_simYaw += speeds.omegaRadiansPerSecond * deltaTime;
      m_simPreviousTimestamp = m_simTimer.get();
    }

    // Update the swerve odometry to the latest position measurements
    m_swerveOdometry.update(getYaw(), getPositions());

    // Update the current module states and chassis speeds
    m_swerveModuleStates = getStates();
    m_chassisSpeeds = Constants.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(m_swerveModuleStates);
  }
}
