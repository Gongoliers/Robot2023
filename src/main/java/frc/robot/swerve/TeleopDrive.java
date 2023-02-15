package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopDrive extends CommandBase {
  private Swerve m_swerve;
  private DoubleSupplier m_translationSupplier;
  private DoubleSupplier m_strafeSupplier;
  private DoubleSupplier m_rotationSupplier;
  private BooleanSupplier m_robotCentricSupplier;

  public TeleopDrive(
      Swerve swerve,
      DoubleSupplier translationSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier robotCentricSupplier) {
    m_swerve = swerve;
    addRequirements(swerve);

    m_translationSupplier = translationSupplier;
    m_strafeSupplier = strafeSupplier;
    m_rotationSupplier = rotationSupplier;
    m_robotCentricSupplier = robotCentricSupplier;
  }

  @Override
  public void execute() {
    double translationScalar =
        MathUtil.applyDeadband(m_translationSupplier.getAsDouble(), Constants.Driver.DEADBAND);
    double strafeScalar =
        MathUtil.applyDeadband(m_strafeSupplier.getAsDouble(), Constants.Driver.DEADBAND);
    double rotationScalar =
        MathUtil.applyDeadband(m_rotationSupplier.getAsDouble(), Constants.Driver.DEADBAND);

    m_swerve.drive(
        new Translation2d(translationScalar, strafeScalar).times(Constants.Swerve.LINEAR_SPEED_MAX),
        new Rotation2d(rotationScalar).times(Constants.Swerve.ANGULAR_SPEED_MAX),
        !m_robotCentricSupplier.getAsBoolean(),
        Constants.Swerve.SHOULD_OPEN_LOOP_IN_TELEOP);
  }
}
