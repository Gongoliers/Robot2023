package frc.robot;

import com.thegongoliers.commands.DoNothingCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.superstructure.Claw;
import frc.robot.superstructure.Extend;
import frc.robot.superstructure.ExtendRetractDistance;
import frc.robot.superstructure.ExtensionController;
import frc.robot.superstructure.Lower;
import frc.robot.superstructure.Raise;
import frc.robot.superstructure.RaiseLowerAngle;
import frc.robot.superstructure.Retract;
import frc.robot.superstructure.RotationController;
import frc.robot.swerve.AbsoluteDrive;
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
  private final Swerve m_swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final Claw m_claw = new Claw();
  private final ExtensionController m_extensionController = new ExtensionController();
  private final RotationController m_rotationController = new RotationController();

  // Controllers
  private final XboxController m_driver = new XboxController(Constants.Driver.CONTROLLER_PORT);
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

    TeleopDrive teleopDrive =
        new TeleopDrive(
            m_swerve,
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(Constants.Driver.LEFT_VERTICAL_AXIS.value),
                    Constants.Driver.DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(Constants.Driver.LEFT_HORIZONTAL_AXIS.value),
                    Constants.Driver.DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(Constants.Driver.RIGHT_HORIZONTAL_AXIS.value),
                    Constants.Driver.DEADBAND),
            () -> false,
            false,
            false);

    AbsoluteDrive absoluteDrive =
        new AbsoluteDrive(
            m_swerve,
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(Constants.Driver.LEFT_VERTICAL_AXIS.value),
                    Constants.Driver.DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(Constants.Driver.LEFT_HORIZONTAL_AXIS.value),
                    Constants.Driver.DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(Constants.Driver.RIGHT_HORIZONTAL_AXIS.value),
                    Constants.Driver.DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(Constants.Driver.RIGHT_VERTICAL_AXIS.value),
                    Constants.Driver.DEADBAND),
            false);

    m_swerve.setDefaultCommand(teleopDrive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Trigger(() -> m_driver.getRawButton(Constants.Driver.ZERO_GYRO_BUTTON.value))
        .onTrue(new InstantCommand(m_swerve::zeroGyro));

    // TODO Test
    new Trigger(() -> m_driver.getRawButton(Constants.Driver.LOCK_BUTTON.value))
        .onTrue(new InstantCommand(m_swerve::lock));

    // TODO Figure our what commands have to go here...
    new Trigger(() -> m_driver.getRawButton(Constants.Driver.PANIC_BUTTON.value))
        .onTrue(new SequentialCommandGroup(new InstantCommand(), new InstantCommand()));

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
        .whileTrue(new Extend(m_extensionController));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.EXTEND_RETRACT_AXIS.value)
                    > Constants.Manipulator.TRIGGER_THRESHOLD)
        .whileTrue(new Retract(m_extensionController));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.RAISE_LOWER_AXIS.value)
                    < -Constants.Manipulator.TRIGGER_THRESHOLD)
        .whileTrue(new Raise(m_rotationController));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.RAISE_LOWER_AXIS.value)
                    > Constants.Manipulator.TRIGGER_THRESHOLD)
        .whileTrue(new Lower(m_rotationController));

    new Trigger(() -> m_manipulator.getRawButton(Constants.Manipulator.FLOOR_BUTTON.value))
        .whileTrue(
            new ParallelCommandGroup(
                new ExtendRetractDistance(
                    m_extensionController, 0.1, Constants.Arm.States.FLOOR.getLength()),
                new RaiseLowerAngle(
                    m_rotationController,
                    0.1,
                    Constants.Arm.States.FLOOR.getAngle().getDegrees())));

    new Trigger(() -> m_manipulator.getRawButton(Constants.Manipulator.MIDDLE_BUTTON.value))
        .whileTrue(
            new ParallelCommandGroup(
                new ExtendRetractDistance(
                    m_extensionController, 0.1, Constants.Arm.States.MIDDLE.getLength()),
                new RaiseLowerAngle(
                    m_rotationController,
                    0.1,
                    Constants.Arm.States.MIDDLE.getAngle().getDegrees())));

    new Trigger(() -> m_manipulator.getRawButton(Constants.Manipulator.TOP_BUTTON.value))
        .whileTrue(
            new ParallelCommandGroup(
                new ExtendRetractDistance(
                    m_extensionController, 0.1, Constants.Arm.States.TOP.getLength()),
                new RaiseLowerAngle(
                    m_rotationController, 0.1, Constants.Arm.States.TOP.getAngle().getDegrees())));
  }

  private void configureTriggers() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new DoNothingCommand();
  }
}
