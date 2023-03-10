// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import com.thegongoliers.math.GMath;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendRetractDistance extends CommandBase {

  private final ExtensionController m_extender;
  private final double m_speed;
  private final double m_setpoint;

  /** Creates a new ManualExtend. */
  public ExtendRetractDistance(ExtensionController extender, double speed, double setpoint) {
    addRequirements(extender);
    m_extender = extender;
    double delta = setpoint - extender.getMeasurement();
    m_speed = Math.copySign(speed, delta);
    m_setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_extender.disable();
    m_extender.unlock();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_extender.drive(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_extender.stop();
    m_extender.lock();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return GMath.approximately(m_extender.getMeasurement(), m_setpoint);
  }
}
