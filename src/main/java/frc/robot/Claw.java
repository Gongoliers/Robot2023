package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Claw {

    Solenoid clawOpen = new Solenoid(PneumaticsModuleType.REVPH, 1);

    public boolean isOpen() {
        return clawOpen.get();
    }

    public void open() {
        clawOpen.set(true);
    }

    public void close() {
        clawOpen.set(false);
    }
}
