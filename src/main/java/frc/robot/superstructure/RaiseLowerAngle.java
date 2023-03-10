// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import com.thegongoliers.math.GMath;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RaiseLowerAngle extends CommandBase {

  private final RotationController m_rotator;
  private final double m_speed;
  private final double m_setpoint;

  /** Creates a new ManualExtend. */
  public RaiseLowerAngle(RotationController rotator, double speed, double setpoint) {
    addRequirements(rotator);
    m_rotator = rotator;
    m_speed = speed;
    m_setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rotator.unlock();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_rotator.drive(m_speed);
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
    return GMath.approximately(m_rotator.getAngle(), m_setpoint);
  }
}
