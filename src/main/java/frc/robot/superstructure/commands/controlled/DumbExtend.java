// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands.controlled;

import com.thegongoliers.math.GMath;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.superstructure.ExtensionController;

public class DumbExtend extends CommandBase {

  private final ExtensionController m_extender;
  private final double m_lengthSetpoint;

  public DumbExtend(ExtensionController extender, double lengthSetpoint) {
    addRequirements(extender);
    m_extender = extender;
    m_lengthSetpoint =
        GMath.clamp(
            lengthSetpoint,
            Constants.Arm.Extension.MIN_EXTENSION_LENGTH,
            Constants.Arm.Extension.MAX_EXTENSION_LENGTH);
  }

  @Override
  public void initialize() {
    m_extender.unlock();
  }

  @Override
  public void execute() {
    if (m_extender.getLength() > m_lengthSetpoint) {
      m_extender.setMotor(Constants.Arm.Extension.MANUAL_RETRACT_SPEED);
    } else {
      m_extender.setMotor(Constants.Arm.Extension.MANUAL_EXTEND_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_extender.stop();
    m_extender.lock();
  }

  private double getLength() {
    return m_extender.getLength();
  }

  private boolean isAtSetpoint() {
      return GMath.approximately(
          getLength(), m_lengthSetpoint, Constants.Arm.Extension.TOLERANCE);
  }

  private boolean isTooShort() {
    return getLength() < Constants.Arm.Extension.MIN_EXTENSION_LENGTH;
  }

  private boolean isTooLong() {
    return getLength() > Constants.Arm.Extension.MAX_EXTENSION_LENGTH;
  }

  @Override
  public boolean isFinished() {
      return isAtSetpoint() || isTooShort() || isTooLong();
  }
}
