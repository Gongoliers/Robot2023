// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands;

import com.thegongoliers.math.GMath;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.superstructure.RotationController;

public class DumbRotate extends CommandBase {

  private final RotationController m_rotator;
  private final double m_angleSetpoint;

  public DumbRotate(RotationController rotator, double angleSetpoint) {
    addRequirements(rotator);
    m_rotator = rotator;
    m_angleSetpoint = angleSetpoint;
  }

  @Override
  public void initialize() {
    m_rotator.unlock();
  }

  @Override
  public void execute() {
    if (m_rotator.getAngle() > m_angleSetpoint) {
      m_rotator.drive(Constants.Arm.Rotation.MANUAL_LOWER_SPEED);
    } else {
      m_rotator.drive(Constants.Arm.Rotation.MANUAL_RAISE_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_rotator.stop();
    m_rotator.lock();
  }

  @Override
  public boolean isFinished() {
    return GMath.approximately(m_rotator.getAngle(), m_angleSetpoint, Constants.Arm.Rotation.TOLERANCE);
  }
}
