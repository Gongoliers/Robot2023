// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SideIntake extends SubsystemBase {
  /** Creates a new Intake. */
  private WPI_TalonSRX m_top, m_bottom;

  public SideIntake() {
    m_top = new WPI_TalonSRX(5);
    m_bottom = new WPI_TalonSRX(6);

    m_top.setInverted(false);
    m_bottom.setInverted(false);
  }

  public void intake() {
    double intakeSpeed = 0.45;
    m_top.set(intakeSpeed);
    m_bottom.set(intakeSpeed);
  }

  public void outtake() {
    double outtakeSpeed = -0.50;
    m_top.set(outtakeSpeed);
    m_bottom.set(outtakeSpeed);
  }

  public void stop() {
    m_top.stopMotor();
    m_bottom.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Top Speed", m_top.get());
    SmartDashboard.putNumber("Bottom Speed", m_bottom.get());
  }
}
