// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.superstructure.RotationController;

public class PIDRotate extends PIDCommand {

  public PIDRotate(RotationController rotator, double angleSetpoint) {
    super(
        new PIDController(
            Constants.Arm.Rotation.KP, Constants.Arm.Rotation.KI, Constants.Arm.Rotation.KD),
        rotator::getAngle,
        angleSetpoint,
        voltage -> rotator.setVoltage(voltage),
        rotator);

    getController().setTolerance(Constants.Arm.Rotation.TOLERANCE);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}