// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands.controlled;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.superstructure.RotationController;

public class PIDRotate extends PIDCommand {

  private final RotationController m_rotator;

  public PIDRotate(RotationController rotator, double angle) {
    super(
        new PIDController(
            Constants.Arm.Rotation.KP, Constants.Arm.Rotation.KI, Constants.Arm.Rotation.KD),
        rotator::getAngle,
        angle,
        speed -> rotator.setMotor(speed),
        rotator);

    m_rotator = rotator;

    getController().setTolerance(Constants.Arm.Rotation.TOLERANCE);
    SmartDashboard.putData(getController());
  }

  @Override
  public void initialize() {
    super.initialize();

    m_rotator.unlock();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    m_rotator.stop();
    m_rotator.lock();
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
