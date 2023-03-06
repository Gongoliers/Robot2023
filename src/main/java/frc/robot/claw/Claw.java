// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.claw;

import com.thegongoliers.output.interfaces.Gripper;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.TelemetrySubsystem;

public class Claw extends SubsystemBase implements Gripper, TelemetrySubsystem {
  public Claw() {}

  @Override
  public void open() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void close() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public boolean isOpen() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void addToShuffleboard(ShuffleboardContainer container) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub
    
  }

}
