package frc.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConfig {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int cancoderID;
  public final Rotation2d angleOffset;
  public final Rotation2d angleStop;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   *
   * @param driveMotorID
   * @param angleMotorID
   * @param canCoderID
   * @param angleOffset
   * @param angleStop
   */
  public SwerveModuleConfig(
      int driveMotorID,
      int angleMotorID,
      int canCoderID,
      Rotation2d angleOffset,
      Rotation2d angleStop) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.cancoderID = canCoderID;
    this.angleOffset = angleOffset;
    this.angleStop = angleStop;
  }
}
