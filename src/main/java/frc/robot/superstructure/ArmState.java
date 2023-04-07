package frc.robot.superstructure;

import frc.robot.Constants;

public enum ArmState {
  STOWED("Stowed", 0.05, Constants.Arm.Rotation.MAX_ANGLE),
  TOP_PIVOT_POINT("Top Pivot", 0.85, 30),
  TOP_CONE("Top Row", 1.075, 18),
  MIDDLE_CONE("Middle Row", 0.45, 7),
  TOP_CUBE("Top Row", 0.8, 25),
  MIDDLE_CUBE("Middle Row", 0.45, 17),
  DOUBLE_SUBSTATION("Double Substation", 0, 13), // TODO
  HYBRID("Floor", 0.1, 0); // TODO

  private final String name;
  public final double length;
  public final double angle;

  /**
   * Constructs an ArmState with the provided extension length and angle.
   *
   * @param name the name of the state.
   * @param length the extension of the arm in meters.
   * @param angle the rotation of the arm in degrees.
   */
  private ArmState(String name, double length, double angle) {
    this.name = name;
    this.length = length;
    this.angle = angle;
  }

  @Override
  public String toString() {
    return name;
  }
}
