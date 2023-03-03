package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class TeleopDrive extends CommandBase {
  private Swerve m_swerve;

  private DoubleSupplier m_vX;
  private DoubleSupplier m_vY;
  private DoubleSupplier m_headingX;
  private DoubleSupplier m_headingY;

  private Translation2d m_velocity;
  private PIDController m_thetaController;
  private Rotation2d m_heading, m_previousHeading, m_omega;

  public TeleopDrive(
      Swerve swerve,
      DoubleSupplier vX,
      DoubleSupplier vY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    m_swerve = swerve;
    addRequirements(swerve);

    m_vX = vX;
    m_vY = vY;
    m_headingX = headingX;
    m_headingY = headingY;
  }

  @Override
  public void initialize() {
    m_thetaController =
        new PIDController(
            Constants.Swerve.THETA_CONTROLLER_KP,
            Constants.Swerve.THETA_CONTROLLER_KI,
            Constants.Swerve.THETA_CONTROLLER_KD);

    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_thetaController.setTolerance(Constants.Swerve.THETA_CONTROLLER_TOLERANCE);

    m_previousHeading = m_swerve.yaw();
  }

  @Override
  public void execute() {
    m_velocity = velocityFromJoystick(m_vX, m_vY);
    m_heading =
        thresholdHeading(headingFromJoystick(m_headingX, m_headingY), Constants.Driver.DEADBAND);
    m_omega = calculateOmega(m_swerve.yaw(), m_heading);

    m_swerve.drive(m_velocity, m_omega, Constants.Swerve.SHOULD_OPEN_LOOP_IN_TELEOP);

    m_previousHeading = m_heading;
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

    // Applies cubic mapping for joysticks 
    vX = Math.pow(vX, 3);
    vY = Math.pow(vY, 3);

    Translation2d velocity = new Translation2d(vX, vY);

    // Scales velocity vector so that full pushes are maximum speed
    velocity = velocity.times(Constants.Swerve.MAX_SPEED);

    // Limits the velocity vector so that it doesn't exceed the maximum speed
    velocity = limitVelocity(velocity);

    return velocity;
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
      return m_previousHeading;
    }

    return heading.getAngle();
  }

  /**
   * Calculates the omega value (angular velocity) in radians per second depending on the measurement and setpoint. If the measurement is within some tolerance of the setpoint, no angular velocity is produced. 
   * @param measurement Rotation2d containing the current measurement. 
   * @param setpoint Rotation2d containing the setpoint. 
   * @return Rotation2d representing the desired angular velocity.
   */
  private Rotation2d calculateOmega(Rotation2d measurement, Rotation2d setpoint) {
    double _measurement = measurement.getRadians();
    double _setpoint = setpoint.getRadians();

    // Don't apply any angular velocity if the error is within tolerance, otherwise
    // calculate the angular velocity needed to reach the heading goal from the heading measurement
    double omega = m_thetaController.atSetpoint() ? 0 : m_thetaController.calculate(_measurement, _setpoint);

    return Rotation2d.fromRadians(omega);
  }

  /**
   * Gets the maximum velocity for the given direction.
   *
   * @param angle the direction to calculate the maximum velocity.
   * @return the maximum velocity that is achievable for this direction.
   */
  private Translation2d getMaxAchievableVelocity(Rotation2d angle) {
    return new Translation2d(Constants.Swerve.MAX_SPEED, angle);
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
