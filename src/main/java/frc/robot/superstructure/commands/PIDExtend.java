// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.superstructure.ExtensionController;

public class PIDExtend extends PIDCommand {

  public PIDExtend(ExtensionController extender, double lengthSetpoint) {
    super(
        new PIDController(
            Constants.Arm.Extension.KP, Constants.Arm.Extension.KI, Constants.Arm.Extension.KD),
        extender::getLength,
        lengthSetpoint,
        voltage -> extender.setVoltage(voltage),
        extender);

    getController().setTolerance(Constants.Arm.Extension.TOLERANCE);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}