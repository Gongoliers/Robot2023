// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands.controlled;

import com.thegongoliers.math.GMath;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.superstructure.RotationController;

public class DumbRotate extends CommandBase {

  private final RotationController m_rotator;
  private final double m_angleSetpoint;

  public DumbRotate(RotationController rotator, double angleSetpoint) {
    addRequirements(rotator);
    m_rotator = rotator;
    m_angleSetpoint =
        GMath.clamp(
            angleSetpoint, Constants.Arm.Rotation.MIN_ANGLE, Constants.Arm.Rotation.MAX_ANGLE);
  }

  @Override
  public void initialize() {
    m_rotator.unlock();
  }

  @Override
  public void execute() {
    if (m_rotator.getAngle() > m_angleSetpoint) {
      m_rotator.drive(Constants.Arm.Rotation.CONTROLLED_LOWER_SPEED);
    } else {
      m_rotator.drive(Constants.Arm.Rotation.CONTROLLED_RAISE_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_rotator.stop();
    m_rotator.lock();
  }

  private double getAngle() {
    return m_rotator.getAngle();
  }

  private boolean isAtSetpoint() {
    return GMath.approximately(
        getAngle(), m_angleSetpoint, Constants.Arm.Rotation.TOLERANCE);
  }

  @Override
  public boolean isFinished() {
    return isAtSetpoint() || isTooLow() || isTooHigh();
  }

  private boolean isTooHigh() {
    return getAngle() > Constants.Arm.Rotation.MAX_ANGLE;
  }

  private boolean isTooLow() {
    return getAngle() > Constants.Arm.Rotation.MIN_ANGLE;
  }
}
