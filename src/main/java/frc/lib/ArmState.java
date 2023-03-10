package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Represents the state of the arm. Implemented as an abstraction over a Translation2d object
 * contructed with a given distance (extension length) and angle.
 */
public class ArmState extends Translation2d {

  /**
   * Constructs an ArmState with an extension length of zero meters and a rotation of zero degrees.
   */
  public ArmState() {}

  /**
   * Constructs an ArmState with the provided extension length and angle.
   *
   * @param meters the extension of the arm in meters.
   * @param degrees the rotation of the arm.
   */
  public ArmState(double meters, Rotation2d degrees) {
    super(meters, degrees);
  }

  /**
   * Returns the extension length of the arm.
   *
   * @return the extension length of the arm.
   */
  public double getLength() {
    return this.getNorm();
  }
}
