// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.thegongoliers.output.interfaces.Stoppable;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.TelemetrySubsystem;

public class RollerClaw extends SubsystemBase implements TelemetrySubsystem, Stoppable {
  
  private final WPI_TalonSRX m_motor;

  public RollerClaw() {
    m_motor = new WPI_TalonSRX(0); // TODO
    configure();

    addToShuffleboard(Shuffleboard.getTab("Superstructure").getLayout("Roller Claw", BuiltInLayouts.kList));
  }

  private void configure() {
    m_motor.setInverted(false); // TODO
  }

  @Override
  public void periodic() {}

  @Override
  public void addToShuffleboard(ShuffleboardContainer container) {
    container.addNumber("Speed", m_motor::get)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -1.0, "max", 1.0));
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub
  }

  public void intake() {
    m_motor.set(0.5); // TODO
  }

  public void hold() {
    m_motor.set(0.05); // TODO
  }

  public void outtake() {
    m_motor.set(-0.5); // TODO
  }

  public void stop() {
    m_motor.stopMotor();
  }
}
