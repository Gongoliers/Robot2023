// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands.controlled;

import com.thegongoliers.math.GMath;
import com.thegongoliers.output.control.BangBangController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.superstructure.ExtensionController;

public class BangBangExtend extends CommandBase {

  private final BangBangController m_bangbangController;
  private final ExtensionController m_extender;

  public BangBangExtend(ExtensionController extender, double lengthSetpoint) {
    addRequirements(extender);
    m_extender = extender;
    m_bangbangController = new BangBangController(Constants.Arm.Extension.MANUAL_EXTEND_SPEED);

    double setpoint =
        GMath.clamp(
            lengthSetpoint,
            Constants.Arm.Extension.MIN_EXTENSION_LENGTH,
            Constants.Arm.Extension.MAX_EXTENSION_LENGTH);

    m_bangbangController.setSetpoint(setpoint);
    m_bangbangController.setTolerance(Constants.Arm.Extension.TOLERANCE);
  }

  @Override
  public void initialize() {
    m_extender.unlock();
  }

  @Override
  public void execute() {
    double measurement = m_extender.getLength();
    // deltaTime is ignored with this implementation of BangBangController
    double speed = m_bangbangController.calculate(measurement, 0.0);
    m_extender.drive(speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_extender.stop();
    m_extender.lock();
  }

  @Override
  public boolean isFinished() {
    return m_bangbangController.atSetpoint();
  }
}
