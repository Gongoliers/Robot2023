package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.swerve.Swerve;
import java.util.List;

public class Auto {

  public static Command fullAuto(Swerve swerve, String pathName, PathConstraints constraints) {

    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, constraints);

    SwerveAutoBuilder autoBuilder =
        new SwerveAutoBuilder(
            swerve::getPose,
            swerve::resetOdometry,
            Constants.Swerve.KINEMATICS,
            Constants.Auto.TRANSLATION_PID,
            Constants.Auto.ROTATION_PID,
            swerve::setModuleStates,
            Constants.Auto.EVENT_MAP,
            true,
            swerve);

    return autoBuilder.fullAuto(pathGroup);
  }
}
