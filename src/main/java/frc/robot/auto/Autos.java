// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.superstructure.Claw;
import frc.robot.superstructure.ExtensionController;
import frc.robot.superstructure.RotationController;
import frc.robot.superstructure.commands.controlled.DumbExtend;
import frc.robot.superstructure.commands.controlled.DumbRotate;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.TeleopDrive;

public final class Autos {

  private final Swerve m_swerve;
  private final ExtensionController m_extensionController;
  private final RotationController m_rotationController;
  private final Claw m_claw;

  public Autos(Swerve swerve, ExtensionController extensionController, RotationController rotationController, Claw claw) {
    m_swerve = swerve;
    m_extensionController = extensionController;
    m_rotationController = rotationController;
    m_claw = claw;
  }

  public Command extendToPosition(double angle, double length) {
    return Commands.sequence(
        new DumbRotate(m_rotationController, angle),
        new DumbExtend(m_extensionController, length));
  }

  public Command retractToPosition(double angle, double length) {
    return Commands.sequence(new DumbExtend(m_extensionController, length), new DumbRotate(m_rotationController, angle));
  }

  public Command stow() {
    return retractToPosition(-5, 0.05);
  }

  public Command score(
      double angle,
      double length) {

      return extendToPosition(angle, length).andThen(m_claw::open, m_claw).andThen(Commands.waitSeconds(0.5)).andThen(m_claw::close, m_claw).andThen(stow());
  }

  public Command scoreTop() {
    return score(-100, 1.1);
  }

  public Command scoreMiddle() {
    return score(
        -130,
        0.58);
  }

  public Command scoreBottom() {
    return score(-200, 0.3);
  }

  public Command driveDistance(double vX, double vY, double distance) {
    return new TeleopDrive(m_swerve, () -> vX, () -> vY, () -> 0, () -> false, false, false)
        .until(() -> Math.abs(m_swerve.getPose().getTranslation().getNorm()) > distance);
  }

  public Command mobility() {
    return driveDistance(0.75, 0.0, 4.0);
  }

  public Command scoreThenMobility(Command scoreCommand) {
    return scoreCommand.andThen(Commands.waitSeconds(0.5)).andThen(mobility());
  }

}
