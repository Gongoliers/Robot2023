// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.superstructure.Claw;
import frc.robot.superstructure.ExtensionController;
import frc.robot.superstructure.RotationController;
import frc.robot.superstructure.commands.controlled.DumbExtend;
import frc.robot.superstructure.commands.controlled.DumbRotate;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.TeleopDrive;
import java.util.List;

public final class Autos {

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(Swerve swerve) {
    List<PathPlannerTrajectory> example1 =
        PathPlanner.loadPathGroup("SamplePath", new PathConstraints(4, 3));

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every
    // time you want
    // to create an auto command. A good place to put this is in RobotContainer along with your
    // subsystems.
    SwerveAutoBuilder autoBuilder =
        new SwerveAutoBuilder(
            swerve::getPose,
            // Pose2d supplier
            swerve::resetOdometry,
            // Pose2d consumer, used to reset odometry at the beginning of auto
            Constants.Auto.TRANSLATION_PID,
            // PID constants to correct for translation error (used to create the X and Y PID
            // controllers)
            Constants.Auto.ROTATION_PID,
            // PID constants to correct for rotation error (used to create the rotation controller)
            swerve::setChassisSpeeds,
            // Module states consumer used to output to the drive subsystem
            Constants.Auto.EVENT_MAP,
            false,
            // Should the path be automatically mirrored depending on alliance color. Optional,
            // defaults to true
            swerve
            // The drive subsystem. Used to properly set the requirements of path following commands
            );
    return Commands.sequence(autoBuilder.fullAuto(example1));
  }

  public static Command scoreTop2(
      ExtensionController extensionController, RotationController rotationController, Claw claw) {

    var m_extensionController = extensionController;
    var m_rotationController = rotationController;
    var m_claw = claw;

    return Commands.sequence(
        new DumbRotate(m_rotationController, -100),
        new DumbExtend(m_extensionController, 1.1),
        new InstantCommand(m_claw::open));
  }

  public static Command score(
      ExtensionController extensionController,
      RotationController rotationController,
      Claw claw,
      double angle,
      double length) {

    var m_extensionController = extensionController;
    var m_rotationController = rotationController;
    var m_claw = claw;

    return Commands.sequence(
        new DumbRotate(m_rotationController, angle),
        new DumbExtend(m_extensionController, length),
        new InstantCommand(m_claw::open),
        new WaitCommand(0.5),
        new InstantCommand(m_claw::close),
        retract(m_extensionController, m_rotationController));
  }

  public static Command scoreTop(
      ExtensionController extensionController, RotationController rotationController, Claw claw) {
    return score(extensionController, rotationController, claw, -100, 1.1);
  }

  public static Command scoreMiddle(
      ExtensionController extensionController, RotationController rotationController, Claw claw) {
    return score(
        extensionController,
        rotationController,
        claw,
        -130,
        0.58); // TODO still needs some tuning, too short?
  }

  public static Command scoreBottom(
      ExtensionController extensionController, RotationController rotationController, Claw claw) {
    return score(extensionController, rotationController, claw, -200, 0.3); // TODO
  }

  public static Command retract(
      ExtensionController extensionController, RotationController rotationController) {
    var m_extensionController = extensionController;
    var m_rotationController = rotationController;
    return Commands.sequence(
        new DumbExtend(m_extensionController, 0.05), new DumbRotate(m_rotationController, -5));
  }

  public static Command backup(Swerve swerve) {
    var m_swerve = swerve;
    return new TeleopDrive(m_swerve, () -> 0.75, () -> 0, () -> 0, () -> false, false, false)
        .until(() -> Math.abs(m_swerve.getPose().getTranslation().getNorm()) > 4.0); // TODO
  }

  public static Command scoreTopBackup(
      ExtensionController ext, RotationController rot, Claw claw, Swerve swerve) {
    var m_ext = ext;
    var m_rot = rot;
    var m_claw = claw;
    return scoreTop(m_ext, m_rot, m_claw).andThen(new WaitCommand(0.5)).andThen(backup(swerve));
  }

  public static Command scoreMiddleBackup(
      ExtensionController ext, RotationController rot, Claw claw, Swerve swerve) {
    var m_ext = ext;
    var m_rot = rot;
    var m_claw = claw;
    return scoreMiddle(m_ext, m_rot, m_claw).andThen(new WaitCommand(0.5)).andThen(backup(swerve));
  }

  public static Command scoreBottomBackup(
      ExtensionController ext, RotationController rot, Claw claw, Swerve swerve) {
    var m_ext = ext;
    var m_rot = rot;
    var m_claw = claw;
    return scoreBottom(m_ext, m_rot, m_claw).andThen(new WaitCommand(0.5)).andThen(backup(swerve));
  }

  public static Command chargeStation(Swerve swerve) {
    var m_swerve = swerve;
    return new TeleopDrive(m_swerve, () -> 0.5, () -> 0, () -> 0, () -> false, false, false)
        .withTimeout(0.25)
        .andThen(new TeleopDrive(m_swerve, () -> 0, () -> 0, () -> 0.5, () -> false, false, false))
        .withTimeout(0.375)
        .andThen(new WaitCommand(1.0))
        .andThen(new TeleopDrive(m_swerve, () -> 0.75, () -> 0.5, () -> 0, () -> false, false, false))
        .until(() -> Math.abs(m_swerve.getPose().getTranslation().getNorm()) > 3.0)
        .andThen(m_swerve::lock, m_swerve); // TODO
  }

  public static Command scoreTopChargeStation(
      ExtensionController ext, RotationController rot, Claw claw, Swerve swerve) {
    var m_ext = ext;
    var m_rot = rot;
    var m_claw = claw;
    return scoreTop(m_ext, m_rot, m_claw)
        .andThen(new WaitCommand(0.5))
        .andThen(chargeStation(swerve));
  }

  public static Command scoreMiddleChargeStation(
      ExtensionController ext, RotationController rot, Claw claw, Swerve swerve) {
    var m_ext = ext;
    var m_rot = rot;
    var m_claw = claw;
    return scoreMiddle(m_ext, m_rot, m_claw)
        .andThen(new WaitCommand(0.5))
        .andThen(chargeStation(swerve));
  }

  public static Command scoreBottomChargeStation(
      ExtensionController ext, RotationController rot, Claw claw, Swerve swerve) {
    var m_ext = ext;
    var m_rot = rot;
    var m_claw = claw;
    return scoreBottom(m_ext, m_rot, m_claw)
        .andThen(new WaitCommand(0.5))
        .andThen(chargeStation(swerve));
  }
}
