// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
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

  public Autos(
      Swerve swerve,
      ExtensionController extensionController,
      RotationController rotationController,
      Claw claw) {
    m_swerve = swerve;
    m_extensionController = extensionController;
    m_rotationController = rotationController;
    m_claw = claw;
  }

  /**
   * Rotate to angle then extend to length. Should be used when the length setpoint is greater than
   * the current length.
   *
   * @param angleSetpoint angle.
   * @param lengthSetpoint length.
   * @return a command which performs this motion.
   */
  public Command extendToPosition(double angleSetpoint, double lengthSetpoint) {
    return Commands.sequence(
        new DumbRotate(m_rotationController, angleSetpoint),
        new DumbExtend(m_extensionController, lengthSetpoint));
  }

  /**
   * Extend to length then rotate to angle. Should be used when the current length is greater than
   * the length setpoint.
   *
   * @param angleSetpoint angle.
   * @param lengthSetpoint length.
   * @return a command which performs this motion.
   */
  public Command retractToPosition(double angleSetpoint, double lengthSetpoint) {
    return Commands.sequence(
        new DumbExtend(m_extensionController, lengthSetpoint),
        new DumbRotate(m_rotationController, angleSetpoint));
  }

  /**
   * Alias for returning the arm to the stow position.
   *
   * @return a command which returns the arm to the stow position.
   */
  public Command stow() {
    return retractToPosition(-5, 0.05);
  }

  /**
   * Extend to setpoint, then drop the game piece, then stow.
   *
   * @param angleSetpoint angle.
   * @param lengthSetpoint length.
   * @return a command which performs this sequence.
   */
  public Command score(double angleSetpoint, double lengthSetpoint) {

    return extendToPosition(angleSetpoint, lengthSetpoint)
        .andThen(m_claw::open, m_claw)
        .andThen(Commands.waitSeconds(0.5))
        .andThen(m_claw::close, m_claw)
        .andThen(stow());
  }

  /**
   * Alias for scoring on the top row.
   *
   * @return a command that scores on the top row.
   */
  public Command scoreTop() {
    return score(-100, 1.1);
  }

  /**
   * Alias for scoring on the middle row.
   *
   * @return a command that scores on the middle row.
   */
  public Command scoreMiddle() {
    return score(-130, 0.58);
  }

  /**
   * Alias for scoring on the bottom row.
   *
   * @return a command that scores on the bottom row.
   */
  public Command scoreBottom() {
    return score(-200, 0.3);
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

  /**
   * Score using the specified score command, then achieve mobility.
   *
   * @param score a command that scores.
   * @return a command that scores, then achieves moblity.
   */
  public Command scoreThenMobility(Command score) {
    return score.andThen(Commands.waitSeconds(0.5)).andThen(mobility());
  }
}
