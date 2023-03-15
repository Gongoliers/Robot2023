// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.thegongoliers.output.interfaces.Stoppable;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.TelemetrySubsystem;
import frc.lib.math.Conversions;
import frc.robot.Constants.GamePiece;

public class Intake extends SubsystemBase implements Stoppable, TelemetrySubsystem {

  private final MotorController m_intake;
  private final WPI_TalonFX m_deployMotor;

  private int m_heldGamePiece = GamePiece.NONE;

  public int getHeldGamePiece() {
    return m_heldGamePiece;
  }

  public void setHeldGamePiece(int gamePiece) {
    m_heldGamePiece = gamePiece;
  }

  public Intake() {
    m_intake = new WPI_TalonFX(999); // TODO
    m_deployMotor = new WPI_TalonFX(999); // TODO
    
    configHardware();
    addToShuffleboard(Shuffleboard.getTab("Intake"));
  }

  private void configHardware() {
    m_intake.setInverted(false);
    
    m_deployMotor.configFactoryDefault();
  }

  public void drive(double speed) {
    m_intake.set(speed);
  }

  public void deployTo(double degrees) {
    double gearRatio = 1.0; // TODO
    double setpoint = Conversions.degreesToFalcon(degrees, gearRatio);
    m_deployMotor.set(ControlMode.Position, setpoint);
  }

  public void stop() {
    m_intake.stopMotor();
    m_deployMotor.stopMotor();
  }

  @Override
  public void periodic() {
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
