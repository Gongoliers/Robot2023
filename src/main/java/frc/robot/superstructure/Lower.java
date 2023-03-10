// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class Lower extends CommandBase {

  private RotationController m_rotator;

  /** Creates a new ManualExtend. */
  public Lower(RotationController rotator) {
    addRequirements(rotator);
    m_rotator = rotator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rotator.unlock();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_rotator.drive(Constants.Arm.Rotation.MANUAL_LOWER_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rotator.stop();
    m_rotator.lock();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_rotator.getAngle() < Constants.Arm.Rotation.MIN_ANGLE;
  }
}
