// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands.manual;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.superstructure.RotationController;

public class SafeRaise extends CommandBase {

  private RotationController m_rotator;

  public SafeRaise(RotationController rotator) {
    addRequirements(rotator);
    m_rotator = rotator;
  }

  @Override
  public void initialize() {
    m_rotator.unlock();
  }

  @Override
  public void execute() {
    m_rotator.drive(Constants.Arm.Rotation.MANUAL_RAISE_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    m_rotator.stop();
    m_rotator.lock();
  }

  @Override
  public boolean isFinished() {
    return m_rotator.isRaised();
  }
}
