package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopDrive extends CommandBase {
  private Swerve swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;

  public TeleopDrive(
      Swerve swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup) {
    this.swerve = swerve;
    addRequirements(swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    // TODO Mikaylee setting
    double translationVal =
        MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Driver.DEADBAND);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Driver.DEADBAND);
    double rotationVal =
        MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Driver.DEADBAND);

    /* Drive */
    swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.LINEAR_SPEED_MAX),
        rotationVal * Constants.Swerve.ANGULAR_SPEED_MAX,
        !robotCentricSup.getAsBoolean(),
        true);
  }
}
