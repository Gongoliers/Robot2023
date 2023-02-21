package frc.robot.swerve;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;

public class SwerveTelemetry {

    private final ShuffleboardTab m_tab;
    private final Swerve m_swerve;

    public SwerveTelemetry(Swerve swerve) {
        m_tab = Shuffleboard.getTab("Swerve");
        m_swerve = swerve;

        heading().withPosition(0, 0);
        odometryX().withPosition(0, 1);
        odometryY().withPosition(0, 2);

        module(0, "Front Left Module").withPosition(1, 0);
        module(1, "Front Right Module").withPosition(2, 0);
        module(2, "Back Left Module").withPosition(3, 0);
        module(3, "Back Right Module").withPosition(4, 0);
    }

    private SuppliedValueWidget<Double> heading() {
        var widget = m_tab.addNumber("Heading", () -> m_swerve.pose().getRotation().getDegrees());
        return widget;
    }
        
    private SuppliedValueWidget<Double> odometryX() {
        var widget = m_tab.addNumber("Odometry X", () -> m_swerve.pose().getX());
        return widget;
    }

    private SuppliedValueWidget<Double> odometryY() {
        var widget = m_tab.addNumber("Odometry Y", () -> m_swerve.pose().getY());
        return widget;
    }

    private ShuffleboardLayout module(int moduleNumber, String moduleName) {
        var layout = m_tab.getLayout(moduleName, BuiltInLayouts.kGrid);
        layout.withProperties(Map.of("Label position", "TOP"));

        SwerveModule module = m_swerve.module(moduleNumber);

        var cancoderAngleWidget = layout.addNumber("CANCoder Angle", () -> module.cancoderAngle().getDegrees());
        cancoderAngleWidget.withPosition(0, 0);

        var encoderAngleWidget = layout.addNumber("Integrated Encoder Angle", () -> module.encoderAngle().getDegrees());
        encoderAngleWidget.withPosition(0, 1);

        var velocityWidget = layout.addNumber("Wheel Velocity", () -> module.state().speedMetersPerSecond);
        velocityWidget.withPosition(0, 2);
        return layout;
    }

}