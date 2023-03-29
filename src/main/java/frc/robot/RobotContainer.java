package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.Autos;
import frc.robot.superstructure.ArmState;
import frc.robot.superstructure.Claw;
import frc.robot.superstructure.ExtensionController;
import frc.robot.superstructure.RotationController;
import frc.robot.superstructure.commands.controlled.DumbRotate;
import frc.robot.superstructure.commands.controlled.PIDRotate;
import frc.robot.superstructure.commands.manual.SafeExtend;
import frc.robot.superstructure.commands.manual.SafeLower;
import frc.robot.superstructure.commands.manual.SafeRaise;
import frc.robot.superstructure.commands.manual.SafeRetract;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.TeleopDrive;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Claw m_claw = new Claw();
  private final RotationController m_rotationController = new RotationController();
  private final ExtensionController m_extensionController =
      new ExtensionController(m_rotationController);

  // Controllers
  //private final XboxController m_driver = new XboxController(Constants.Driver.CONTROLLER_PORT);
  private final XboxController m_manipulator =
      new XboxController(Constants.Manipulator.CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    setDefaultCommands();
    configureButtonBindings();
    configureTriggers();
  }

  /**
   * The default command will be automatically scheduled when no other commands are scheduled that
   * require the subsystem.
   */
  private void setDefaultCommands() {
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.CLOSE_AXIS.value)
                    > Constants.Manipulator.TRIGGER_THRESHOLD)
        .onTrue(new InstantCommand(m_claw::close));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.OPEN_AXIS.value)
                    > Constants.Manipulator.TRIGGER_THRESHOLD)
        .onTrue(new InstantCommand(m_claw::open));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.EXTEND_RETRACT_AXIS.value)
                    < -Constants.Manipulator.TRIGGER_THRESHOLD)
        .whileTrue(new SafeExtend(m_extensionController));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.EXTEND_RETRACT_AXIS.value)
                    > Constants.Manipulator.TRIGGER_THRESHOLD)
        .whileTrue(new SafeRetract(m_extensionController));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.RAISE_LOWER_AXIS.value)
                    < -Constants.Manipulator.TRIGGER_THRESHOLD)
        .whileTrue(new SafeRaise(m_rotationController));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.RAISE_LOWER_AXIS.value)
                    > Constants.Manipulator.TRIGGER_THRESHOLD)
        .whileTrue(new SafeLower(m_rotationController));

    new Trigger(() -> m_manipulator.getRawButton(Constants.Manipulator.FLOOR_BUTTON.value))
        // .whileTrue(new DumbRotate(m_rotationController, ArmState.FLOOR.getAngle()));
        .whileTrue(new PIDRotate(m_rotationController, ArmState.FLOOR.angle));

    new Trigger(() -> m_manipulator.getRawButton(Constants.Manipulator.LEVEL_BUTTON.value))
        //.whileTrue(new DumbRotate(m_rotationController, ArmState.STOWED.getAngle()));
        .whileTrue(new PIDRotate(m_rotationController, ArmState.STOWED.angle));

    new Trigger(() -> m_manipulator.getRawButton(Constants.Manipulator.TOP_BUTTON.value))
        //.whileTrue(new DumbRotate(m_rotationController, ArmState.TOP.getAngle()));
        .whileTrue(new PIDRotate(m_rotationController, ArmState.TOP.angle));

    new Trigger(() -> m_manipulator.getRawButton(Constants.Manipulator.SUBSTATION_BUTTON.value))
        //.whileTrue(new DumbRotate(m_rotationController, ArmState.DOUBLE_SUBSTATION.getAngle()));
        .whileTrue(new PIDRotate(m_rotationController, ArmState.DOUBLE_SUBSTATION.angle));
  }

  private void configureTriggers() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}