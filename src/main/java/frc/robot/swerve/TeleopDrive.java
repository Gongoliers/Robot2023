package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class TeleopDrive extends CommandBase {
  private Swerve m_swerve;
  private DoubleSupplier m_translationSupplier;
  private DoubleSupplier m_strafeSupplier;
  private DoubleSupplier m_rotationSupplier;

  public TeleopDrive(
      Swerve swerve,
      DoubleSupplier translationSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier rotationSupplier) {
    m_swerve = swerve;
    addRequirements(swerve);

    m_translationSupplier = translationSupplier;
    m_strafeSupplier = strafeSupplier;
    m_rotationSupplier = rotationSupplier;
  }

  @Override
  public void execute() {
    double translation = m_translationSupplier.getAsDouble();
    double strafe = m_strafeSupplier.getAsDouble();

    Translation2d velocity =
        new Translation2d(translation, strafe).times(Constants.Swerve.LINEAR_SPEED_MAX);
    velocity = limitVelocity(velocity);

    Rotation2d rotation =
        new Rotation2d(m_rotationSupplier.getAsDouble()).times(Constants.Swerve.ANGULAR_SPEED_MAX);

    m_swerve.drive(velocity, rotation, Constants.Swerve.SHOULD_OPEN_LOOP_IN_TELEOP);
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
