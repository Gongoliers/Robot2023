package frc.robot.swerve;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import java.util.Map;

public class SwerveTelemetry {

  public static void createShuffleboardTab(Swerve swerve, String tabName) {
    ShuffleboardTab tab = Shuffleboard.getTab(tabName);
    addHeading(swerve, tab).withPosition(0, 0);
    addOdometryX(swerve, tab).withPosition(0, 1);
    addOdometryY(swerve, tab).withPosition(0, 2);

    addModule(swerve, tab, 0, "Front Left Module").withPosition(1, 0);
    addModule(swerve, tab, 1, "Front Right Module").withPosition(2, 0);
    addModule(swerve, tab, 2, "Back Left Module").withPosition(3, 0);
    addModule(swerve, tab, 3, "Back Right Module").withPosition(4, 0);
  }

  private static SuppliedValueWidget<Double> addHeading(Swerve swerve, ShuffleboardTab tab) {
    var widget = tab.addNumber("Heading", () -> swerve.pose().getRotation().getDegrees());
    return widget;
  }

  private static SuppliedValueWidget<Double> addOdometryX(Swerve swerve, ShuffleboardTab tab) {
    var widget = tab.addNumber("Odometry X", () -> swerve.pose().getX());
    return widget;
  }

  private static SuppliedValueWidget<Double> addOdometryY(Swerve swerve, ShuffleboardTab tab) {
    var widget = tab.addNumber("Odometry Y", () -> swerve.pose().getY());
    return widget;
  }

  private static ShuffleboardLayout addModule(
      Swerve swerve, ShuffleboardTab tab, int moduleNumber, String moduleName) {
    var layout = tab.getLayout(moduleName, BuiltInLayouts.kGrid);
    layout.withProperties(Map.of("Label position", "TOP"));

    SwerveModule module = swerve.module(moduleNumber);

    var cancoderAngleWidget =
        layout.addNumber("CANCoder Angle", () -> module.cancoderAngle().getDegrees());
    cancoderAngleWidget.withPosition(0, 0);

    var encoderAngleWidget =
        layout.addNumber("Integrated Encoder Angle", () -> module.encoderAngle().getDegrees());
    encoderAngleWidget.withPosition(0, 1);

    var velocityWidget =
        layout.addNumber("Wheel Velocity", () -> module.state().speedMetersPerSecond);
    velocityWidget.withPosition(0, 2);
    return layout;
  }
}
