package frc.robot.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Auto;
import frc.robot.swerve.Swerve;

public class FollowTrajectory extends SequentialCommandGroup {

  public FollowTrajectory(Swerve swerve, PathPlannerTrajectory trajectory, boolean resetOdometry) {
    addRequirements(swerve);

    if (resetOdometry) {
      swerve.resetOdometry(trajectory.getInitialHolonomicPose());
    }

    addCommands(
        new PPSwerveControllerCommand(
            trajectory,
            swerve::getPose,
            new PIDController(
                Auto.TRANSLATION_PID.kP, Auto.TRANSLATION_PID.kI, Auto.TRANSLATION_PID.kD),
            new PIDController(
                Auto.TRANSLATION_PID.kP, Auto.TRANSLATION_PID.kI, Auto.TRANSLATION_PID.kD),
            new PIDController(Auto.ROTATION_PID.kP, Auto.ROTATION_PID.kI, Auto.ROTATION_PID.kD),
            swerve::setChassisSpeeds,
            swerve));
  }
}
