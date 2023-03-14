// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import com.thegongoliers.output.interfaces.Gripper;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.TelemetrySubsystem;
import frc.robot.Constants;

public class Claw extends SubsystemBase implements Gripper, TelemetrySubsystem {

  private Solenoid m_solenoid;

  public Claw() {
    m_solenoid =
        new Solenoid(
            Constants.PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, Constants.Claw.CHANNEL);

    addToShuffleboard(Shuffleboard.getTab("Superstructure").getLayout("Claw", BuiltInLayouts.kList).withSize(1, 2).withPosition(4, 0));
  }

  @Override
  public void open() {
    m_solenoid.set(true);
  }

  @Override
  public void close() {
    m_solenoid.set(false);
  }

  @Override
  public boolean isOpen() {
    return m_solenoid.get();
  }

  @Override
  public void addToShuffleboard(ShuffleboardContainer container) {
    container.addBoolean("Claw Open?", this::isOpen);
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub

  }
}
