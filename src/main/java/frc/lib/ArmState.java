package frc.lib;

public enum ArmState {

  STOWED("Stowed", 0, 0), // TODO
  TOP("Top Row", 0, 0), // TODO
  DOUBLE_SUBSTATION("Double Substation", 0, 0), // TODO
  FLOOR("Floor", 0, 0); // TODO

  private final String name;
  private final double length;
  private final double angle;

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

  public double getLength() {
    return length;
  }

  public double getAngle() {
    return angle;
  }

}
