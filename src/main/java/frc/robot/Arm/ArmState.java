package frc.robot.Arm;

public class ArmState {
    enum name {
        TOP_SETPOINT,
        MIDDLE_SETPOINT,
        HYBRID_SETPOINT,
    }

    
    double currentAngle;
    double currentExtension;
}
