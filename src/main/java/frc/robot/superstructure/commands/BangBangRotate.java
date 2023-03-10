// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands;

import com.thegongoliers.output.control.BangBangController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.superstructure.RotationController;

public class BangBangRotate extends CommandBase {

  private final BangBangController m_bangbangController;
  private final RotationController m_rotator;

  public BangBangRotate(RotationController extender, double angleSetpoint) {
    addRequirements(extender);
    m_rotator = extender;
    m_bangbangController = new BangBangController(Constants.Arm.Rotation.MANUAL_RAISE_SPEED);

    m_bangbangController.setSetpoint(angleSetpoint);
    m_bangbangController.setTolerance(Constants.Arm.Rotation.TOLERANCE);
  }

  @Override
  public void initialize() {
    m_rotator.unlock();
  }

  @Override
  public void execute() {
    double measurement = m_rotator.getAngle();
    // deltaTime is ignored with this implementation of BangBangController
    double speed = m_bangbangController.calculate(measurement, 0.0);
    m_rotator.drive(speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_rotator.stop();
    m_rotator.lock();
  }

  @Override
  public boolean isFinished() {
    return m_bangbangController.atSetpoint();
  }
}
