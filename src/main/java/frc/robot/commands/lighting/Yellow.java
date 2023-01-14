package frc.robot.commands.lighting;

import frc.robot.subsystems.LightingSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Yellow extends InstantCommand {

    private final LightingSubsystem m_lightingSubsytem;

    public Yellow(LightingSubsystem lightingSubsystem) {
        this.m_lightingSubsytem = lightingSubsystem;

        addRequirements(lightingSubsystem);
    }

    
    @Override
    public void initialize() {
        m_lightingSubsytem.yellow();
    }
}
