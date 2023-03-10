// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class Extend extends CommandBase {

  private ExtensionController m_extender;

  /** Creates a new ManualExtend. */
  public Extend(ExtensionController extender) {
    addRequirements(extender);
    m_extender = extender;
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
    m_extender.drive(Constants.Arm.Extension.MANUAL_EXTEND_SPEED);
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
    return m_extender.getMeasurement() > Constants.Arm.Extension.MAX_EXTENSION_LENGTH;
  }
}
