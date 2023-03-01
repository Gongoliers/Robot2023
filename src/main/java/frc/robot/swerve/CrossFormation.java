// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CrossFormation extends CommandBase {

  private final Swerve m_swerve;

  public CrossFormation(Swerve swerve) {
    m_swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    m_swerve.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(0, Constants.Swerve.FRONT_LEFT_MODULE.ANGLE_STOP),
      new SwerveModuleState(0, Constants.Swerve.FRONT_RIGHT_MODULE.ANGLE_STOP),
      new SwerveModuleState(0, Constants.Swerve.BACK_LEFT_MODULE.ANGLE_STOP),
      new SwerveModuleState(0, Constants.Swerve.BACK_RIGHT_MODULE.ANGLE_STOP),
    });
  }
}
