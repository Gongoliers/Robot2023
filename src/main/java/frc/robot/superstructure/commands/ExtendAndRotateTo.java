// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.superstructure.ExtensionController;
import frc.robot.superstructure.RotationController;
import frc.robot.superstructure.commands.controlled.DumbExtend;
import frc.robot.superstructure.commands.controlled.DumbRotate;

public class ExtendAndRotateTo extends ParallelCommandGroup {
  public ExtendAndRotateTo(
      double angle, double length, ExtensionController extender, RotationController rotator) {
    // TODO Test PID strategy
    // addCommands(
    //     new PIDExtend(extender, desiredState.getLength()),
    //     new PIDRotate(rotator, desiredState.getAngle().getDegrees()));

    // TODO Test BangBang strategy
    // addCommands(
    //     new BangBangExtend(extender, desiredState.getLength()),
    //     new BangBangRotate(rotator, desiredState.getAngle().getDegrees()));

    // TODO Test Dumb strategy
    addCommands(new DumbExtend(extender, length), new DumbRotate(rotator, angle));

    // TODO Test a career that doesn't involve programming...
  }
}
