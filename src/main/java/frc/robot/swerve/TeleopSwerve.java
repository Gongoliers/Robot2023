package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;

  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

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
        MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.OperatorInput.STICK_DEADBAND)
            * 0.3;
    double strafeVal =
        MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.OperatorInput.STICK_DEADBAND)
            * 0.3;
    double rotationVal =
        MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.OperatorInput.STICK_DEADBAND)
            * 0.3;

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_LINEAR_SPEED),
        rotationVal * Constants.Swerve.MAX_ANGULAR_SPEED,
        !robotCentricSup.getAsBoolean(),
        true);
  }
}
