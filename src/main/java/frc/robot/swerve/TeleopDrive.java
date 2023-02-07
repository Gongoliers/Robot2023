package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopDrive extends CommandBase {
  private Swerve swerve;
  private DoubleSupplier translationSupplier;
  private DoubleSupplier strafeSupplier;
  private DoubleSupplier rotationSupplier;
  private BooleanSupplier robotCentricSupplier;

  public TeleopDrive(
      Swerve swerve,
      DoubleSupplier translationSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier robotCentricSupplier) {
    this.swerve = swerve;
    addRequirements(swerve);

    this.translationSupplier = translationSupplier;
    this.strafeSupplier = strafeSupplier;
    this.rotationSupplier = rotationSupplier;
    this.robotCentricSupplier = robotCentricSupplier;
  }

  @Override
  public void execute() {
    // TODO Mikaylee setting
    double translation =
        MathUtil.applyDeadband(translationSupplier.getAsDouble(), Constants.Driver.DEADBAND);
    double strafe = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.Driver.DEADBAND);
    double rotation =
        MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Constants.Driver.DEADBAND);

    /* Drive */
    swerve.drive(
        new Translation2d(translation, strafe).times(Constants.Swerve.LINEAR_SPEED_MAX),
        new Rotation2d(rotation).times(Constants.Swerve.ANGULAR_SPEED_MAX),
        !robotCentricSupplier.getAsBoolean(),
        Constants.Swerve.SHOULD_OPEN_LOOP_IN_TELEOP);
  }
}
