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
import frc.robot.Constants;
import frc.robot.superstructure.Claw;
import frc.robot.superstructure.ExtensionController;
import frc.robot.superstructure.RotationController;
import frc.robot.superstructure.commands.controlled.DumbExtend;
import frc.robot.superstructure.commands.controlled.DumbRotate;
import frc.robot.swerve.Swerve;
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

  public static Command scoreTop(
      ExtensionController m_extensionController,
      RotationController m_rotationController,
      Claw m_claw) {

    return Commands.sequence(
        new DumbRotate(m_rotationController, -100),
        new DumbExtend(m_extensionController, 1.1),
        new InstantCommand(m_claw::open));
  }
}
