// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands.controlled;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.superstructure.ExtensionController;

public class PIDExtend extends PIDCommand {

  private final ExtensionController m_extender;

  public PIDExtend(ExtensionController extender, double length) {
    super(
        new PIDController(
            Constants.Arm.Extension.KP, Constants.Arm.Extension.KI, Constants.Arm.Extension.KD),
        extender::getLength,
        length,
        speed -> extender.setMotor(speed),
        extender);

    m_extender = extender;

    getController().setTolerance(Constants.Arm.Extension.TOLERANCE);
    SmartDashboard.putData(getController());
  }

  @Override
  public void initialize() {
    super.initialize();
    
    m_extender.unlock();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    m_extender.stop();
    m_extender.lock();
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
