package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Claw {
    private Solenoid m_clawState = new Solenoid(PneumaticsModuleType.REVPH, 1);

    /**
     * Get the state of the claw.
     * @return true if the claw is open, false if the claw is closed.
     */
    public boolean ClawState() {
        return m_clawState.get();
    }

    /**
     * Set claw state to open.
     */
    public void open() {
        m_clawState.set(true);
    }

    /**
     * Set claw state to closed.
     */
    public void close() {
        m_clawState.set(false);
    }
}
