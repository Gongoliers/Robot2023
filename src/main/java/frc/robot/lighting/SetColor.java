package frc.robot.lighting;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetColor extends CommandBase {

    private final Lighting m_lighting;
    private final Color m_color;

    public SetColor(Lighting lighting, Color color) {
        m_lighting = lighting;
        addRequirements(lighting);

        m_color = color;
    }
   
    @Override
    public void initialize() {
        m_lighting.clear();
        m_lighting.setColor(m_color);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
