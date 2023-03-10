// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import com.thegongoliers.output.interfaces.Extendable;
import com.thegongoliers.output.interfaces.Lockable;
import com.thegongoliers.output.interfaces.Retractable;
import com.thegongoliers.output.interfaces.Stoppable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase implements Stoppable, Lockable, Extendable, Retractable {

  private ExtensionController m_extensionController;
  private RotationController m_rotationController;

  public Arm() {
    m_extensionController = new ExtensionController();
    m_rotationController = new RotationController();
  }

  @Override
  public void retract() {
    // FIXME
  }

  @Override
  public boolean isRetracted() {
    // FIXME
    return true;
  }

  @Override
  public void extend() {
    // FIXME
  }

  @Override
  public boolean isExtended() {
    // FIXME
    return true;
  }

  @Override
  public void lock() {
    m_extensionController.lock();
    m_rotationController.lock();
  }

  @Override
  public void unlock() {
    m_extensionController.unlock();
    m_rotationController.unlock();
  }

  @Override
  public void stop() {
    m_extensionController.stop();
    m_rotationController.stop();
  }
}