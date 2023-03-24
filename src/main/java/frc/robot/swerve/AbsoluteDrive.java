// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class AbsoluteDrive extends CommandBase {

  private final Swerve m_swerve;
  private final DoubleSupplier m_vX, m_vY;
  private final DoubleSupplier m_headingHorizontal, m_headingVertical;

  public AbsoluteDrive(Swerve swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingHorizontal, DoubleSupplier headingVertical) {
    m_swerve = swerve;
    m_vX = vX;
    m_vY = vY;
    m_headingHorizontal = headingHorizontal;
    m_headingVertical = headingVertical;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Convert joystick inputs to desired chassis speeds
    ChassisSpeeds desiredSpeeds = m_swerve.getTargetSpeeds(m_vX.getAsDouble(), m_vY.getAsDouble(), m_headingHorizontal.getAsDouble(), m_headingVertical.getAsDouble());

    // Get the translational velocity component of the desired chassis speeds
    Translation2d desiredVelocity = SwerveController.getTranslation2d(desiredSpeeds);
    // TODO Test
    // Limit the velocity to prevent tipping
    desiredVelocity = SwerveMath.limitVelocity(desiredVelocity, m_swerve.getFieldVelocity(), m_swerve.getPose(), Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS), m_swerve.getSwerveDriveConfiguration()) ;

    // Add debug info to SmartDashboard
    SmartDashboard.putNumber("DesiredLimitedVelocity", desiredVelocity.getX());
    SmartDashboard.putString("DesiredVelocity", desiredVelocity.toString());

    // Drive the robot using the desired translational velocity and rotational velocity
    m_swerve.drive(desiredVelocity, desiredSpeeds.omegaRadiansPerSecond, true, true);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
