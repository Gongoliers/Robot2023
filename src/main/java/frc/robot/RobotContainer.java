package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.exampleAuto;
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

  // Controllers
  private final Joystick m_driverController = new Joystick(Constants.Driver.CONTROLLER_PORT);

  // Driver buttons
  private final Trigger m_zeroGyro =
      new Trigger(() -> m_driverController.getRawButton(Constants.Driver.BUTTON_ZERO_GYRO.value));
  private final Trigger m_robotCentric =
      new Trigger(
          () -> m_driverController.getRawButton(Constants.Driver.BUTTON_ROBOT_CENTRIC.value));
  private final Trigger m_turbo =
      new Trigger(
          () ->
              m_driverController.getRawAxis(Constants.Driver.AXIS_TURBO_MODE.value)
                  > Constants.Driver.TRIGGER_THRESHOLD);

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
            () -> -m_driverController.getRawAxis(Constants.Driver.AXIS_TRANSLATION.value),
            () -> -m_driverController.getRawAxis(Constants.Driver.AXIS_STRAFE.value),
            () -> -m_driverController.getRawAxis(Constants.Driver.AXIS_ROTATION.value),
            () -> m_robotCentric.getAsBoolean()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    m_zeroGyro.onTrue(m_swerve.zeroGyro());
    m_turbo.onTrue(m_swerve.enableTurbo()).onFalse(m_swerve.disableTurbo());
  }

  private void configureTriggers() {
  }

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
