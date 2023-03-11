// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands.manual;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.superstructure.ExtensionController;

public class SafeRetract extends CommandBase {

  private ExtensionController m_extender;

  public SafeRetract(ExtensionController extender) {
    addRequirements(extender);
    m_extender = extender;
  }

  @Override
  public void initialize() {
    m_extender.unlock();
  }

  @Override
  public void execute() {
    m_extender.drive(Constants.Arm.Extension.CONTROLLED_RETRACT_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    m_extender.stop();
    m_extender.lock();
  }

  @Override
  public boolean isFinished() {
    return m_extender.getLength() < Constants.Arm.Extension.MIN_EXTENSION_LENGTH;
  }
}
