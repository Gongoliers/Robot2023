// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands;

import com.thegongoliers.math.GMath;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.superstructure.ExtensionController;

public class ExtendRetractDistance extends CommandBase {

  private final ExtensionController m_extender;
  private final double m_speed;
  private final double m_setpoint;

  public ExtendRetractDistance(ExtensionController extender, double speed, double setpoint) {
    addRequirements(extender);
    m_extender = extender;
    m_speed = speed;
    m_setpoint = setpoint;
  }

  @Override
  public void initialize() {
    m_extender.unlock();
  }

  @Override
  public void execute() {
    m_extender.drive(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_extender.stop();
    m_extender.lock();
  }

  @Override
  public boolean isFinished() {
    return GMath.approximately(m_extender.getLength(), m_setpoint);
  }
}