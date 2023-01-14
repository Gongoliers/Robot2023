package frc.robot.commands.lighting;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LightingSubsystem;

public class Red extends InstantCommand {

  private final LightingSubsystem m_lightingSubsytem;

  public Red(LightingSubsystem lightingSubsystem) {
    this.m_lightingSubsytem = lightingSubsystem;

    addRequirements(lightingSubsystem);
  }

  @Override
  public void initialize() {
    m_lightingSubsytem.red();
  }
}
