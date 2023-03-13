// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands.manual;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.superstructure.ExtensionController;

public class SafeExtend extends CommandBase {

  private ExtensionController m_extender;

  public SafeExtend(ExtensionController extender) {
    addRequirements(extender);
    m_extender = extender;
  }

  @Override
  public void initialize() {
    m_extender.unlock();
  }

  @Override
  public void execute() {
    m_extender.drive(Constants.Arm.Extension.MANUAL_EXTEND_SPEED);
    SmartDashboard.putNumber("Actual Length", m_extender.getLength());
    SmartDashboard.putNumber("Threshold", m_extender.getMaxLength());
    SmartDashboard.putBoolean("Length", m_extender.getLength() > m_extender.getMaxLength());
    SmartDashboard.putBoolean("Lock", m_extender.isLocked());
  }

  @Override
  public void end(boolean interrupted) {
    m_extender.stop();
    m_extender.lock();
  }

  @Override
  public boolean isFinished() {
    return m_extender.getLength() > m_extender.getMaxLength(); // || m_extender.isLocked();
  }
}
