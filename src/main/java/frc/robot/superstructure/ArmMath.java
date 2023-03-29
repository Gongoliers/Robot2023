// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import java.util.Collections;
import java.util.List;

import frc.robot.Constants;

public class ArmMath {

  /**
   * Calculates the maximum arm length that can be achieved at the specified angle without exceeding the specified horizontal measurement.
   * @param angle the arm angle, in degrees.
   * @param horizontal the horizontal measurement, in meters.
   * @return the maximum arm length that fulfills this constraint, in meters.
   */
  public static double calculateHorizontalLengthConstraint(double angle, double horizontal) {
    double cos = Math.cos(angle * Math.PI / 180);
  
    // If cosine (horizontal component) is 0, this means that the arm is vertical
    // If the arm is vertical, the horizontal measurement will never be exceeded, so this constraint is not a factor
    if (cos == 0) {
      return Double.POSITIVE_INFINITY;
    }

    // Otherwise, return the maximum arm length that fulfills this constraint
    return horizontal / cos;
  }

  /**
   * Calculates the maximum arm length that can be achieved at the specified angle without exceeding the specified vertical measurement.
   * @param angle the arm angle, in degrees.
   * @param vertical the vertical measurement, in meters.
   * @return the maximum arm length that fulfills this constraint, in meters.
   */
  public static double calculateVerticalLengthConstraint(double angle, double vertical) {
    // TODO
    return Double.POSITIVE_INFINITY;
  }

  public static double calculateLengthConstraint(double angle) {
    double length = Constants.Arm.Extension.MAX_EXTENSION_LENGTH;
    double horizontal = calculateHorizontalLengthConstraint(angle, Constants.Arm.Extension.MAX_HORIZONTAL_LENGTH);
    double vertical = calculateVerticalLengthConstraint(angle, Constants.Arm.Extension.MAX_VERTICAL_EXTENSION);

    List<Double> constraints = List.of(length, horizontal, vertical);

    return Collections.min(constraints);
  }

}