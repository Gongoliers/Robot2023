package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class TeleopDrive extends CommandBase {
  private Swerve m_swerve;

  private PIDController thetaDegreesController;

  private DoubleSupplier m_velocityX;
  private DoubleSupplier m_velocityY;
  private DoubleSupplier m_headingX;
  private DoubleSupplier m_headingY;

  private double vX, vY;
  private Rotation2d angle, lastAngle, omega;

  public TeleopDrive(
      Swerve swerve,
      DoubleSupplier velocityX,
      DoubleSupplier velocityY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    m_swerve = swerve;
    addRequirements(swerve);

    m_velocityX = velocityX;
    m_velocityY = velocityY;
    m_headingX = headingX;
    m_headingY = headingY;
  }

  @Override
  public void initialize() {
    thetaDegreesController = new PIDController(1, 0, 0);
    thetaDegreesController.enableContinuousInput(-180, 180);
    thetaDegreesController.setTolerance(5);
    lastAngle = m_swerve.yaw();
  }

  @Override
  public void execute() {
    vX = m_velocityX.getAsDouble();
    vY = m_velocityY.getAsDouble();

    Translation2d velocity = new Translation2d(vX, vY).times(Constants.Swerve.LINEAR_SPEED_MAX);
    velocity = limitVelocity(velocity);

    Translation2d desiredAngle =
        new Translation2d(m_headingY.getAsDouble(), -m_headingX.getAsDouble());

    if (desiredAngle.getNorm() < 0.5) {
      angle = lastAngle;
    } else {
      angle = desiredAngle.getAngle();
    }

    // Angular velocity
    double omegaDegrees =
        thetaDegreesController.calculate(m_swerve.yaw().getDegrees(), angle.getDegrees());
    omega = Rotation2d.fromDegrees(omegaDegrees).times(Constants.Swerve.ANGULAR_SPEED_MAX);

    m_swerve.drive(velocity, omega, Constants.Swerve.SHOULD_OPEN_LOOP_IN_TELEOP);

    lastAngle = angle;
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
    final Rotation2d kVelocityAngle = desiredVelocity.getAngle();

    final Translation2d kMaxAchievableVelocity = getMaxAchievableVelocity(kVelocityAngle);

    // Limit the velocity if it exceeds the maximum
    if (desiredVelocity.getNorm() > kMaxAchievableVelocity.getNorm()) {
      return kMaxAchievableVelocity;
    }

    return desiredVelocity;
  }
}
