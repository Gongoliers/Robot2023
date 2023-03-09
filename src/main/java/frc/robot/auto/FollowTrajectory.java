package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.swerve.Swerve;

public class FollowTrajectory extends SequentialCommandGroup {
  public FollowTrajectory(Swerve swerve, Trajectory trajectory) {
    var thetaController =
        new ProfiledPIDController(
            Constants.Auto.THETA_CONTROLLER_KP, 0, 0, Constants.Auto.THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            swerve::getPose,
            Constants.Swerve.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.X_CONTROLLER_KP, 0, 0),
            new PIDController(Constants.Auto.Y_CONTROLLER_KP, 0, 0),
            thetaController,
            swerve::setModuleStates,
            swerve);

    addCommands(
        new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand);
  }
}
