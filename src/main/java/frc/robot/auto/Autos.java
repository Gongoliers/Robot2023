// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.superstructure.ArmState;
import frc.robot.superstructure.ExtensionController;
import frc.robot.superstructure.RollerClaw;
import frc.robot.superstructure.RotationController;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.TeleopDrive;

public final class Autos {

  private final Swerve m_swerve;
  private final ExtensionController m_extensionController;
  private final RotationController m_rotationController;
  private final RollerClaw m_claw;

  public Autos(
      Swerve swerve,
      ExtensionController extensionController,
      RotationController rotationController,
      RollerClaw claw) {
    m_swerve = swerve;
    m_extensionController = extensionController;
    m_rotationController = rotationController;
    m_claw = claw;
  }

  public Command extendToPosition(ArmState state) {
    return Commands.sequence(
        m_rotationController.rotateTo(state),
        m_extensionController.extendTo(state));
  }

  public Command retractToPosition(ArmState state) {
    return Commands.sequence(
        m_extensionController.extendTo(state),
        m_rotationController.rotateTo(state));
  }

  /**
   * Alias for returning the arm to the stow position.
   *
   * @return a command which returns the arm to the stow position.
   */
  public Command stow() {
    return retractToPosition(ArmState.STOWED);
  }

  public Command score(ArmState state) {
    return extendToPosition(state)
        .andThen(m_claw::outtake, m_claw)
        .andThen(Commands.waitSeconds(0.5))
        .andThen(m_claw::stop, m_claw)
        .andThen(stow());
  }

  /**
   * Alias for scoring on the top row.
   *
   * @return a command that scores on the top row.
   */
  public Command scoreTop() {
    return score(ArmState.TOP);
  }

  /**
   * Alias for scoring on the middle row.
   *
   * @return a command that scores on the middle row.
   */
  public Command scoreMiddle() {
    return score(ArmState.MIDDLE);
  }

  /**
   * Alias for scoring on the bottom row.
   *
   * @return a command that scores on the bottom row.
   */
  public Command scoreBottom() {
    return score(ArmState.FLOOR);
  }

  /**
   * Reset the swerve drive odometry to (0, 0).
   *
   * @return a command that resets the swerve drive odometry.
   */
  public Command resetPose() {
    // TODO Call m_swerve.zeroGyro()?
    return Commands.runOnce(() -> m_swerve.resetOdometry(new Pose2d()), m_swerve);
  }

  /**
   * Drive with the specified robot-centric velocities for the specified distance.
   *
   * @param vX robot-centric X velocity.
   * @param vY robot-centric Y velocity.
   * @param distance distance to drive.
   * @return a command that drives the specified distance.
   */
  public Command driveDistance(double vX, double vY, double distance) {
    return resetPose()
        .andThen(new TeleopDrive(m_swerve, () -> vX, () -> vY, () -> 0, () -> false, false, false))
        .until(() -> Math.abs(m_swerve.getPose().getTranslation().getNorm()) >= distance);
  }

  /**
   * Alias for achieving mobility.
   *
   * @return a command that achieves mobility.
   */
  public Command mobility() {
    return driveDistance(0.75, 0.0, 4.0);
  }
}
