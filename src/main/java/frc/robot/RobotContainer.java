package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.exampleAuto;
import frc.robot.superstructure.Claw;
import frc.robot.superstructure.ExtensionController;
import frc.robot.superstructure.RotationController;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.TeleopDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Swerve m_swerve = new Swerve();
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
    m_swerve.setDefaultCommand(
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
            () ->
                MathUtil.applyDeadband(
                    m_driver.getRawAxis(Constants.Driver.RIGHT_VERTICAL_AXIS.value),
                    Constants.Driver.DEADBAND)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Trigger(() -> m_driver.getRawButton(Constants.Driver.ZERO_GYRO_BUTTON.value))
        .onTrue(new InstantCommand(m_swerve::setYawZero));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.CLOSE_AXIS.value)
                    > Constants.Manipulator.TRIGGER_THRESHOLD)
        .onTrue(new InstantCommand(() -> m_claw.close()));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.OPEN_AXIS.value)
                    > Constants.Manipulator.TRIGGER_THRESHOLD)
        .onTrue(new InstantCommand(() -> m_claw.open()));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.EXTEND_RETRACT_AXIS.value)
                    < -Constants.Manipulator.TRIGGER_THRESHOLD)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_extensionController.unlock();
                  m_extensionController.drive(Constants.Arm.MANUAL_EXTEND_SPEED);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  m_extensionController.lock();
                }));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.EXTEND_RETRACT_AXIS.value)
                    > Constants.Manipulator.TRIGGER_THRESHOLD)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_extensionController.unlock();
                  m_extensionController.drive(Constants.Arm.MANUAL_RETRACT_SPEED);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  m_extensionController.lock();
                }));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.RAISE_LOWER_AXIS.value)
                    < -Constants.Manipulator.TRIGGER_THRESHOLD)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_rotationController.unlock();
                  m_rotationController.drive(Constants.Arm.MANUAL_RAISE_SPEED);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  m_rotationController.lock();
                }));

    new Trigger(
            () ->
                m_manipulator.getRawAxis(Constants.Manipulator.RAISE_LOWER_AXIS.value)
                    > Constants.Manipulator.TRIGGER_THRESHOLD)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_rotationController.unlock();
                  m_rotationController.drive(Constants.Arm.MANUAL_LOWER_SPEED);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  m_rotationController.lock();
                }));
  }

  private void configureTriggers() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(m_swerve);
  }
}
