package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmState {
  /** The extension (in meters) from the zero point of the arm. */
  public double extension;
  /** The rotation (in degrees) from the zero point of the arm. */
  public Rotation2d angle = Rotation2d.fromDegrees(0);

  /**
   * Construct an ArmState object with an extension of zero meters and a rotation of zero meters.
   * Used for returning the arm to a neutral position.
   */
  public ArmState() {}

  /**
   * Construct an ArmState object with a custom extension and angle.
   *
   * @param extension the extension of the arm in meters.
   * @param angle the rotation of the arm.
   */
  public ArmState(double extension, Rotation2d angle) {
    this.extension = extension;
    this.angle = angle;
  }
}
