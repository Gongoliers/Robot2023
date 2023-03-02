package frc.robot.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class TeleopDrive extends CommandBase {
  private Swerve m_swerve;

  private DoubleSupplier m_desiredVelocityX;
  private DoubleSupplier m_desiredVelocityY;
  private DoubleSupplier m_desiredHeadingCos;
  private DoubleSupplier m_desiredHeadingSin;

  private Translation2d m_velocity;
  private ProfiledPIDController m_thetaController;
  private Rotation2d m_heading, m_angularVelocity;

  public TeleopDrive(
      Swerve swerve,
      DoubleSupplier desiredVelocityX,
      DoubleSupplier desiredVelocityY,
      DoubleSupplier desiredHeadingCos,
      DoubleSupplier desiredHeadingSin) {
    m_swerve = swerve;
    addRequirements(swerve);

    m_desiredVelocityX = desiredVelocityX;
    m_desiredVelocityY = desiredVelocityY;
    m_desiredHeadingCos = desiredHeadingCos;
    m_desiredHeadingSin = desiredHeadingSin;
  }

  @Override
  public void initialize() {
    m_thetaController =
        new ProfiledPIDController(
            Constants.Swerve.THETA_CONTROLLER_KP,
            Constants.Swerve.THETA_CONTROLLER_KI,
            Constants.Swerve.THETA_CONTROLLER_KD,
            Constants.Swerve.THETA_CONTROLLER_CONSTRAINTS);

    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_thetaController.setTolerance(Constants.Swerve.THETA_CONTROLLER_TOLERANCE);
  }

  @Override
  public void execute() {
    m_velocity = velocityFromJoystick(m_desiredVelocityX, m_desiredVelocityY);
    m_heading =
        headingFromJoystick(m_desiredHeadingCos, m_desiredHeadingSin, Constants.Driver.DEADBAND);

    // Current measurement of the process variable
    double headingMeasurement = m_swerve.yaw().getRadians();
    // Desired measurement; "goal"
    double headingGoal = m_heading.getRadians();

    // Calculate the angular velocity needed to reach the heading goal from the heading measurement
    double angularVelocity = m_thetaController.calculate(headingMeasurement, headingGoal);

    m_angularVelocity = Rotation2d.fromRadians(angularVelocity);

    m_swerve.drive(m_velocity, m_angularVelocity, Constants.Swerve.SHOULD_OPEN_LOOP_IN_TELEOP);
  }

  /**
   * Transforms the joystick values from the joystick to actual X and Y velocities.
   *
   * @param velocityX DoubleSupplier for the X velocity value.
   * @param velocityY DoubleSupplier for the Y velocity value.
   * @return Translation2d representing the desired velocity.
   */
  private Translation2d velocityFromJoystick(DoubleSupplier velocityX, DoubleSupplier velocityY) {
    // Inverts X velocity and Y velocity
    double vX = -velocityX.getAsDouble();
    double vY = -velocityY.getAsDouble();

    Translation2d velocity = new Translation2d(vX, vY);

    // Scales velocity vector so that full pushes are maximum speed
    velocity = velocity.times(Constants.Swerve.LINEAR_SPEED_MAX);

    // Limits the velocity vector so that it doesn't exceed the maximum speed
    velocity = limitVelocity(velocity);

    return velocity;
  }

  /**
   * Transforms the heading cosine and sine values from the joystick into actual cosine and sine
   * values. Performs a mapping between the ENWS (East-North-West-South) headings of the joystick to
   * the NESW (North-East-South-West) headings of the robot. Also thresholds the joystick heading so
   * that only large displacements cause the actual heading to change. If the joystick displacement
   * does not pass the threshold, the previous heading is returned.
   *
   * @param headingCos DoubleSupplier for the cosine value.
   * @param headingSin DoubleSupplier for the sine value.
   * @param threshold the amount of displacement required to register a change.
   * @return Rotation2d representing the desired heading.
   */
  private Rotation2d headingFromJoystick(
      DoubleSupplier headingCos, DoubleSupplier headingSin, double threshold) {
    Translation2d heading = headingFromJoystick(headingCos, headingSin);
    return thresholdHeading(heading, threshold);
  }

  /**
   * Transforms the heading cosine and sine values from the joystick into actual cosine and sine
   * values. Performs a mapping between the ENWS (East-North-West-South) headings of the joystick to
   * the NESW (North-East-South-West) headings of the robot.
   *
   * @param headingCos DoubleSupplier for the cosine value.
   * @param headingSin DoubleSupplier for the sine value.
   * @return Translation2d representing the desired heading and the desired magnitude.
   */
  private Translation2d headingFromJoystick(DoubleSupplier headingCos, DoubleSupplier headingSin) {
    // Inverts east-west (has the effect of swapping east and west)
    double cos = -headingCos.getAsDouble();
    // Keeps north-south as it is
    double sin = headingSin.getAsDouble();

    Translation2d heading = new Translation2d(cos, sin);

    // Shifts headings counterclockwise (left)
    heading = heading.rotateBy(Rotation2d.fromRadians(Math.PI / 2));
    return heading;
  }

  /**
   * Thresholds a joystick heading so that only large displacements cause the actual heading to
   * change. If the joystick displacement does not pass the threshold, the previous heading is
   * returned.
   *
   * @param heading Translation2d representing the heading and the joystick displacement.
   * @param threshold the amount of displacement required to register a change.
   * @return Rotation2d representing the desired heading.
   */
  private Rotation2d thresholdHeading(Translation2d heading, double threshold) {
    if (heading.getNorm() < threshold) {
      return m_swerve.yaw();
    }

    return heading.getAngle();
  }

  /**
   * Gets the maximum velocity for the given direction.
   *
   * @param angle the direction to calculate the maximum velocity.
   * @return the maximum velocity that is achievable for this direction.
   */
  private Translation2d getMaxAchievableVelocity(Rotation2d angle) {
    return new Translation2d(Constants.Swerve.LINEAR_SPEED_MAX, angle);
  }

  /**
   * Limits a desired velocity to prevent exceeding the maximum velocity constraint.
   *
   * @param desiredVelocity the desired velocity.
   * @return The velocity after limiting, which is either the desired velocity, or the maximum
   *     velocity in that direction.
   */
  private Translation2d limitVelocity(Translation2d desiredVelocity) {
    Rotation2d velocityAngle = desiredVelocity.getAngle();

    Translation2d maxAchievableVelocity = getMaxAchievableVelocity(velocityAngle);

    // Limit the velocity if it exceeds the maximum
    if (desiredVelocity.getNorm() > maxAchievableVelocity.getNorm()) {
      return maxAchievableVelocity;
    }

    return desiredVelocity;
  }
}
