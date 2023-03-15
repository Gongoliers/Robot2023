// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;

public class Hold extends CommandBase {
  
  private Intake m_intake;

  public Hold(Intake intake) {
    addRequirements(intake);
    m_intake = intake;
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.getHeldGamePiece() == GamePiece.CUBE) {
      m_intake.drive(0); // TODO
    } else if (m_intake.getHeldGamePiece() == GamePiece.CONE) {
      m_intake.drive(0); // TODO
    } else {
      m_intake.drive(0); // TODO
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
