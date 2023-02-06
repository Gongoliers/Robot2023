package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.swerve.Swerve;
import java.util.List;

public class exampleAuto extends SequentialCommandGroup {
  public exampleAuto(Swerve s_Swerve) {
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.Auto.LINEAR_SPEED_MAX, Constants.Auto.LINEAR_ACCELERATION_MAX)
            .setKinematics(Constants.Swerve.SWERVE_KINEMATICS);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            Constants.Auto.THETA_CONTROLLER_KP, 0, 0, Constants.Auto.THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            s_Swerve::pose,
            Constants.Swerve.SWERVE_KINEMATICS,
            new PIDController(Constants.Auto.X_CONTROLLER_KP, 0, 0),
            new PIDController(Constants.Auto.Y_CONTROLLER_KP, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    addCommands(
        // new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
        // swerveControllerCommand
        );
  }
}
